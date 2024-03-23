#include <stdio.h>
#include <driver/gpio.h>
#include <math.h>
#include <string.h>
#include <sys/unistd.h>
#include <esp_task.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"

/*
 * init statuses: receive status is no, receive not finished
 * receive status -> yes && receive not finish, block send
 * trigger receive mode ->
 *      case1(receive status is no) set receive status to yes && set receive not finished && create receive task
 *      case2(receive status is yes) do nothing
 * receive task finished -> set receive status -> no && set receive finished
 * trigger cancel receive ->
 *      case1(receive status is no) do nothing
 *      case2(receive status is yes) set receive status to no && set receive not finished && delete receive task
 * trigger send ->
 *      case1(receive status is no) send
 *      case2(receive status is yes) do not send
 * trigger save ->
 *      case1(receive status is no && receive finished) save
 *      case2(receive status is yes) do nothing
 *
 */
#define EXAMPLE_IR_RESOLUTION_HZ     (1*1000*1000) // 1MHz resolution, 1 tick = 1us
#define EXAMPLE_IR_TX_GPIO_NUM       18
#define EXAMPLE_IR_RX_GPIO_NUM       22
#define DEFAULT_SSID "GL-MT3000-25f"
#define DEFAULT_PWD "zxk123456"
#define WIFI_STATUS_GPIO_NUM 4
#define REQUEST_GPIO_NUM 2
#define TMT_STATUS_GPIO_NUM 16
#define STORAGE_NAMESPACE "storage"
#define RECEIVE_MODE 1
#define SEND_MODE 0
#define OTHER_MODE 2
#define FINISHED 1
#define NOT_FINISHED 0
#define MAX_IR_NAME_LENGTH 25

static const char *TAG = "WIFI_CONTROL";
static rmt_channel_handle_t tx_channel = NULL;
static rmt_channel_handle_t rx_channel = NULL;
static TaskHandle_t rmt_receive_task_handle = NULL;
static TaskHandle_t rmt_send_task_handle = NULL;
static uint8_t led_level = 0x0;
static size_t symbol_num = 300;
static rmt_symbol_word_t *total_avg;
static size_t ir_mode = 0; // 0: send, 1: receive, 2: other
static size_t receive_status = 0; // 0: start/not finished, 1: finished

static esp_err_t save_ir_signal(rmt_symbol_word_t *symbols, size_t length, const char *key);

static esp_err_t get_ir_signal(rmt_symbol_word_t **symbols, size_t *length, const char *key);

static void ir_send(rmt_symbol_word_t *symbols, size_t length);

static void ir_receiver_task(void *pvParameters);

static void configure_led(void);

static bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data);

static esp_err_t save_ir_handler(httpd_req_t *req);

static esp_err_t send_ir_handler(httpd_req_t *req);

static esp_err_t receive_ir_handler(httpd_req_t *req);

static esp_err_t cancel_receive_ir_handler(httpd_req_t *req);

static esp_err_t index_handler(httpd_req_t *req);

static void configure_ir_tx();

static void configure_ir_rx();

static void configure_wifi();

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

static void initialize_nvs();

static void configure_http_server();

static httpd_uri_t index_html = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
};
static httpd_uri_t save_ir_action = {
        .uri       = "/ir/save",
        .method    = HTTP_GET,
        .handler   = save_ir_handler,
};

static httpd_uri_t send_ir_action = {
        .uri       = "/ir/send",
        .method    = HTTP_GET,
        .handler   = send_ir_handler,
};
static httpd_uri_t receive_ir_action = {
        .uri       = "/ir/receive",
        .method    = HTTP_GET,
        .handler   = receive_ir_handler,
};
static httpd_uri_t cancel_receive_ir_action = {
        .uri       = "/ir/receive/cancel",
        .method    = HTTP_GET,
        .handler   = cancel_receive_ir_handler,
};
static const httpd_uri_t *handlers[] = {
        &save_ir_action,
        &send_ir_action,
        &receive_ir_action,
        &cancel_receive_ir_action,
        &index_html
};

void app_main() {
    configure_led();
    initialize_nvs();
    configure_wifi();
    configure_http_server();
}

static void initialize_nvs() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

static void configure_ir_rx() {
    rmt_rx_channel_config_t rx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,   // 选择时钟源
            .resolution_hz = 1 * 1000 * 1000, // 1 MHz 滴答分辨率，即 1 滴答 = 1 µs
            .mem_block_symbols = 256,          // 内存块大小，即 64 * 4 = 256 字节
            .gpio_num = EXAMPLE_IR_RX_GPIO_NUM,                    // GPIO 编号
            .flags.invert_in = false,         // 不反转输入信号
            .flags.with_dma = false,          // 不需要 DMA 后端
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_channel));
}

static void configure_ir_tx() {
    rmt_tx_channel_config_t tx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,   // 选择时钟源
            .gpio_num = EXAMPLE_IR_TX_GPIO_NUM,                    // GPIO 编号
            .mem_block_symbols = 256,          // 内存块大小，即 256 * 4 = 1024 字节
            .resolution_hz = EXAMPLE_IR_RESOLUTION_HZ, // 1 MHz 滴答分辨率，即 1 滴答 = 1 µs
            .trans_queue_depth = 4,           // 设置后台等待处理的事务数量
            .flags.invert_out = false,        // 不反转输出信号
            .flags.with_dma = false,          // 不需要 DMA 后端
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_channel));

    rmt_carrier_config_t tx_carrier_cfg = {
            .duty_cycle = 0.33f,                 // 载波占空比为 33%
            .frequency_hz = 38000,              // 38 KHz
            .flags.polarity_active_low = false, // 载波应调制到高电平
    };
    // 将载波调制到 TX 通道
    ESP_ERROR_CHECK(rmt_apply_carrier(tx_channel, &tx_carrier_cfg));
}

static void configure_wifi(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    // Initialize default station as network interface instance (esp-netif)
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    esp_netif_set_hostname(sta_netif, "IR_Remote_Control");
    // Initialize and start WiFi
    wifi_config_t wifi_config = {
            .sta = {
                    .ssid = DEFAULT_SSID,
                    .password = DEFAULT_PWD,
                    .scan_method = WIFI_FAST_SCAN,
                    .sort_method = WIFI_FAST_SCAN,
                    .threshold.rssi = -127,
                    .threshold.authmode = WIFI_AUTH_OPEN,
            },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void configure_http_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server;
    if (httpd_start(&server, &config) == ESP_OK) {
        for (int i = 0; handlers[i]; i++) {
            httpd_register_uri_handler(server, handlers[i]);
        }
    }
}

static void configure_led(void) {
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(TMT_STATUS_GPIO_NUM);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(TMT_STATUS_GPIO_NUM, GPIO_MODE_OUTPUT);

    gpio_reset_pin(WIFI_STATUS_GPIO_NUM);
    gpio_set_direction(WIFI_STATUS_GPIO_NUM, GPIO_MODE_OUTPUT);

    gpio_reset_pin(REQUEST_GPIO_NUM);
    gpio_set_direction(REQUEST_GPIO_NUM, GPIO_MODE_OUTPUT);
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        /* Set the GPIO level according to the state (LOW or HIGH)*/
        gpio_set_level(WIFI_STATUS_GPIO_NUM, 0);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        /* Set the GPIO level according to the state (LOW or HIGH)*/
        gpio_set_level(WIFI_STATUS_GPIO_NUM, 1);
    }
}

static void ir_send(rmt_symbol_word_t *symbols, size_t length) {
    configure_ir_tx();
    ESP_ERROR_CHECK(rmt_enable(tx_channel));
    rmt_transmit_config_t transmit_config = {
            .loop_count = 0, // 不循环
    };
    rmt_copy_encoder_config_t raw_config = {};
    rmt_encoder_handle_t raw_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&raw_config, &raw_encoder));
    ESP_LOGI(TAG, "start to send the average value");
    ESP_LOGI(TAG, "===============================================================");
    ESP_LOGI(TAG, "send length: %d", length);
    ESP_ERROR_CHECK(
            rmt_transmit(tx_channel,
                         raw_encoder,
                         symbols,
                         length * sizeof(rmt_symbol_word_t),
                         &transmit_config)
    );
    rmt_tx_wait_all_done(tx_channel, portMAX_DELAY);
    rmt_disable(tx_channel);
    rmt_del_channel(tx_channel);
    ESP_LOGI(TAG, "send the average value done");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

static void ir_receiver_task(void *pvParameters) {
    configure_ir_rx();
    ESP_ERROR_CHECK(rmt_enable(rx_channel));
    QueueHandle_t receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_event_callbacks_t cbs = {
            .on_recv_done = rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, receive_queue));
    // 以下时间要求均基于 NEC 协议
    rmt_receive_config_t receive_config = {
            .signal_range_min_ns = 1250,     // NEC 信号的最短持续时间为 560 µs，由于 1250 ns < 560 µs，有效信号不会视为噪声
            .signal_range_max_ns = 50 * 1000 * 1000, // NEC 信号的最长持续时间为 9000 µs，由于 12000000 ns > 9000 µs，接收不会提前停止
            .flags.en_partial_rx = false,    // 不需要部分接收
    };

    rmt_symbol_word_t raw_symbols[256]; // 为接收的符号分配内存

    // 准备开始接收
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
    // 等待 RX 完成信号
    rmt_rx_done_event_data_t rx_data;

    int learning_times = 4;
    int current_learning_times = 0;
    rmt_symbol_word_t total_raw_symbols[learning_times][symbol_num];
    uint8_t is_inited = 0;
    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if (!is_inited && xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(1000)) == pdPASS) {
            ESP_LOGI(TAG, "init symbol_length: %d", rx_data.num_symbols);
            symbol_num = rx_data.num_symbols;
            is_inited = 1;
            ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
        } else {
            if (current_learning_times < learning_times &&
                xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(1000)) == pdPASS) {
                if (symbol_num != rx_data.num_symbols) {
                    ESP_LOGI(TAG, "symbol_length not match, current symbol_length is %d, please send again",
                             rx_data.num_symbols);
                    ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
                    continue;
                }
                ESP_LOGI(TAG, "waiting for receive, start learning times: %d", current_learning_times + 1);
                rmt_symbol_word_t *receivedSymbols = rx_data.received_symbols;
                for (int i = 0; i < rx_data.num_symbols; ++i) {
                    total_raw_symbols[current_learning_times][i] = receivedSymbols[i];
                }
                current_learning_times++;
                if (current_learning_times == learning_times) {
                    continue;
                }
                ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
            }
            if (current_learning_times == learning_times) {
                printf("\r\n");
                total_avg = (rmt_symbol_word_t *) malloc(sizeof(rmt_symbol_word_t) * symbol_num);

                uint32_t duration0s[symbol_num];
                uint32_t duration1s[symbol_num];
                for (int i = 0; i < symbol_num; ++i) {
                    duration0s[i] = 0;
                    duration1s[i] = 0;
                }
                for (int i = 0; i < learning_times; ++i) {
                    for (int j = 0; j < symbol_num; ++j) {
                        duration0s[j] += total_raw_symbols[i][j].duration0;
                        duration1s[j] += total_raw_symbols[i][j].duration1;
                    }
                }
                for (int i = 0; i < symbol_num; ++i) {
                    duration0s[i] /= learning_times;
                    duration1s[i] /= learning_times;
                }

                for (int i = 0; i < symbol_num; ++i) {
                    total_avg[i].duration0 = duration0s[i];
                    total_avg[i].duration1 = duration1s[i];
                    total_avg[i].level0 = 1;
                    total_avg[i].level1 = 0;
                    printf("%d,%d ", total_avg[i].duration0, total_avg[i].duration1);
                }
                printf("\r\n");
                ESP_LOGI(TAG, "learning done");
                receive_status = FINISHED;
                ir_mode = OTHER_MODE;
                if (rx_channel != NULL) {
                    rmt_disable(rx_channel);
                    rmt_del_channel(rx_channel);
                    rx_channel = NULL;
                }
                vTaskDelete(NULL);
            }
        }
    }
}

static esp_err_t save_ir_signal(rmt_symbol_word_t *symbols, size_t length, const char *key) {
    nvs_handle_t my_handle;
    esp_err_t err;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    printf("sss:  %s\r\n", key);
    err = nvs_get_blob(my_handle, key, NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    uint32_t ir_durations[length * 2 + 1];
    ir_durations[0] = length;
    for (int i = 1; i < length + 1; ++i) {
        ir_durations[i * 2 - 1] = symbols[i - 1].duration0;
        ir_durations[i * 2] = symbols[i - 1].duration1;
        printf("%d,%d ", symbols[i - 1].duration0, symbols[i - 1].duration1);
    }
    printf("\r\n");

    err = nvs_set_blob(my_handle, key, ir_durations, (length * 2 + 1) * sizeof(uint32_t));
    if (err != ESP_OK) return err;

    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    nvs_close(my_handle);
    return ESP_OK;
}

typedef struct {
    int id;
    char name[MAX_IR_NAME_LENGTH + 1];
} IRItem;

// 将结构体数组序列化为字节序列
static void serialize_struct_array(IRItem *array, size_t length, uint8_t **data, size_t *size) {
    *size = sizeof(IRItem) * length;
    *data = (uint8_t *) malloc(*size);
    memcpy(*data, array, *size);
}

// 将字节序列反序列化为结构体数组
static void deserialize_struct_array(uint8_t *data, size_t size, IRItem **array, size_t *length) {
    *length = size / sizeof(IRItem);
    *array = (IRItem *) malloc(size);
    memcpy(*array, data, size);
}

// 保存结构体数组到NVS
static esp_err_t save_struct_array_to_nvs(const char *key, IRItem *array, size_t length, nvs_handle_t my_handle) {
    uint8_t *data;
    size_t size;
    serialize_struct_array(array, length, &data, &size);
    esp_err_t err = nvs_set_blob(my_handle, key, data, size);
    free(data);
    return err;
}

// 从NVS中加载结构体数组
esp_err_t load_struct_array_from_nvs(const char *key, IRItem **array, size_t *length, nvs_handle_t my_handle) {
    size_t size;
    uint8_t *data;
    esp_err_t err = nvs_get_blob(my_handle, key, NULL, &size);
    if (err != ESP_OK) return err;
    data = (uint8_t *) malloc(size);
    err = nvs_get_blob(my_handle, key, data, &size);
    if (err != ESP_OK) {
        free(data);
        return err;
    }
    deserialize_struct_array(data, size, array, length);

    free(data);
    return ESP_OK;
}

static esp_err_t save_ir_index_handler(IRItem ir_item) {
    nvs_handle_t my_handle;
    esp_err_t err;
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    err = nvs_get_blob(my_handle, "ir_index_table", NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
    if (required_size != 0) {
        IRItem *items;
        size_t length;
        err = load_struct_array_from_nvs("ir_index_table", &items, &length, my_handle);
        if (err != ESP_OK) {
            free(items);
            return err;
        }

        for (int i = 0; i < length; ++i) {
            printf("pre %s ", items[i].name);
        }
        printf("\r\n");
        IRItem *new_items = (IRItem *) malloc(sizeof(IRItem) * (length + 1));
        for (int i = 0; i < length; ++i) {
            strncpy(new_items[i].name, items[i].name, MAX_IR_NAME_LENGTH);
            new_items[i].name[MAX_IR_NAME_LENGTH] = '\0';
            new_items[i].id = items[i].id;
        }
        strncpy(new_items[length].name, ir_item.name, MAX_IR_NAME_LENGTH);
        new_items[length].name[MAX_IR_NAME_LENGTH] = '\0';
        new_items[length].id = ir_item.id;
        free(items);
        err = save_struct_array_to_nvs("ir_index_table", new_items, length + 1, my_handle);
        if (err != ESP_OK) {
            free(new_items);
            return err;
        }
        free(new_items);
    } else {
        err = save_struct_array_to_nvs("ir_index_table", &ir_item, 1, my_handle);
        if (err != ESP_OK) {
            return err;
        }
    }

    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    nvs_close(my_handle);
    return ESP_OK;
}

static esp_err_t get_ir_signal(rmt_symbol_word_t **symbols, size_t *length, const char *key) {
    nvs_handle_t my_handle;
    esp_err_t err;
    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read run time blob
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    // obtain required memory space to store blob being read from NVS
    err = nvs_get_blob(my_handle, key, NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
    if (required_size == 0) {
        printf("Nothing saved yet!\r\n");
    } else {
        uint32_t *ir_symbols = (uint32_t *) malloc(required_size);
        err = nvs_get_blob(my_handle, key, ir_symbols, &required_size);
        if (err != ESP_OK) {
            free(ir_symbols);
            return err;
        }
        ESP_LOGI(TAG, "ir_symbols: %lu", ir_symbols[0]);
        *length = ir_symbols[0];
        *symbols = (rmt_symbol_word_t *) malloc(sizeof(rmt_symbol_word_t) * ir_symbols[0]);
        for (int i = 1; i < ir_symbols[0] + 1; ++i) {
            printf("%lu,%lu ", ir_symbols[i * 2 - 1], ir_symbols[i * 2]);
            if (i + 1 % 20 == 0) {
                printf("\r\n");
            }
            (*symbols)[i - 1].duration0 = ir_symbols[i * 2 - 1];
            (*symbols)[i - 1].duration1 = ir_symbols[i * 2];
            (*symbols)[i - 1].level0 = 1;
            (*symbols)[i - 1].level1 = 0;
        }
        printf("\r\n");
        free(ir_symbols);
    }
    nvs_close(my_handle);
    return ESP_OK;
}

static bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {

    BaseType_t high_task_wakeup = pdFALSE;
    //set led on
    led_level = ~led_level;
    gpio_set_level(TMT_STATUS_GPIO_NUM, led_level);

    QueueHandle_t receive_queue = (QueueHandle_t) user_data;
    // 将接收到的 RMT 符号发送到解析任务的消息队列中
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    // 返回是否唤醒了任何任务
    return high_task_wakeup == pdTRUE;
}


static esp_err_t save_ir_handler(httpd_req_t *req) {
    if (ir_mode != RECEIVE_MODE && receive_status == FINISHED) {
        gpio_set_level(REQUEST_GPIO_NUM, 1);
        char *buf;
        size_t buf_len;
        buf_len = httpd_req_get_url_query_len(req) + 1;
        ESP_LOGI(TAG, "%d", buf_len);
        if (buf_len > 1) {
            buf = (char *) malloc(buf_len);
            if (!buf) {
                httpd_resp_send_500(req);
                return ESP_FAIL;
            }
            if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
                char param[20];
                if (httpd_query_key_value(buf, "name", param, sizeof(param)) == ESP_OK) {
                    IRItem ir_item;
                    ir_item.id = 1;
                    strncpy(ir_item.name, param, MAX_IR_NAME_LENGTH);
                    ir_item.name[MAX_IR_NAME_LENGTH] = '\0';
                    save_ir_index_handler(ir_item);
                    save_ir_signal(total_avg, symbol_num, param);
                    ESP_LOGI(TAG, "%s", param);
                    httpd_resp_send(req, param, strlen(param));
                } else {
                    httpd_resp_send_404(req);
                }
            } else {
                httpd_resp_send_404(req);
            }
            free(buf);
        } else {
            httpd_resp_send_404(req);
        }
        return ESP_OK;
    }

    char *res_message = "Receive not finished or not in Receive IR Data Mode";
    httpd_resp_send(req, res_message, strlen(res_message));
    return ESP_OK;
}

static esp_err_t send_ir_handler(httpd_req_t *req) {
    if (ir_mode == RECEIVE_MODE) {
        char *res_message = "Receive IR Data";
        httpd_resp_send(req, res_message, strlen(res_message));
        return ESP_OK;
    }
    ir_mode = SEND_MODE;

    gpio_set_level(REQUEST_GPIO_NUM, 0);
    rmt_symbol_word_t *symbols;
    size_t length = 0;


    get_ir_signal(&symbols, &length, "ioi");
    if (symbols == NULL) {
        char *res_message = "Send IR Data";
        httpd_resp_send(req, res_message, strlen(res_message));
        return ESP_OK;
    }
    ir_send(symbols, length);
    free(symbols);
    httpd_resp_send(req, "Send success", strlen("Send success"));
    ir_mode = OTHER_MODE;
    return ESP_OK;
}

static esp_err_t receive_ir_handler(httpd_req_t *req) {
    if (ir_mode == RECEIVE_MODE) {
        char *res_message = "Receive IR Data";
        httpd_resp_send(req, res_message, strlen(res_message));
        return ESP_OK;
    }
    ir_mode = RECEIVE_MODE;
    receive_status = NOT_FINISHED;
    xTaskCreatePinnedToCore(ir_receiver_task, "rmt_receive_task", 8192 * 2, NULL, 5, &rmt_receive_task_handle, 1);
    char *res_message = "Receive IR Data";
    httpd_resp_send(req, res_message, strlen(res_message));
    return ESP_OK;
}

static esp_err_t cancel_receive_ir_handler(httpd_req_t *req) {
    char *res_message = "Not in Receive IR Data Mode";
    if (ir_mode == RECEIVE_MODE) {
        receive_status = NOT_FINISHED;
        ir_mode = OTHER_MODE;
        if (rx_channel != NULL) {
            rmt_disable(rx_channel);
            rmt_del_channel(rx_channel);
            rx_channel = NULL;
        }
        ESP_LOGI(TAG, "Cancel Receive IR Data");
        vTaskDelete(rmt_receive_task_handle);

        res_message = "Cancel Receive IR Data";
    }
    httpd_resp_send(req, res_message, strlen(res_message));
    return ESP_OK;
}

static esp_err_t index_handler(httpd_req_t *req) {
    nvs_handle_t my_handle;
    esp_err_t err;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK) return err;
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    err = nvs_get_blob(my_handle, "ir_index_table", NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
    IRItem *items;
    size_t length;
    char *html = strdup("<ul>");
    if (html == NULL) {
        return ESP_ERR_NO_MEM; // 处理内存分配失败的情况
    }
    if (required_size != 0) {
        err = load_struct_array_from_nvs("ir_index_table", &items, &length, my_handle);
        if (err != ESP_OK) {
            free(html);
            free(items);
            return err;
        }
        for (size_t i = 0; i < length; ++i) {
            char *li = NULL;
            ESP_LOGI(TAG, "%s", items[i].name);
            ESP_LOGI(TAG, "%d", items[i].name[0]);
            ESP_LOGI(TAG, "%d", items[i].name[1]);
            ESP_LOGI(TAG, "%d", items[i].name[2]);
            ESP_LOGI(TAG, "%d", items[i].name[3]);
            asprintf(&li, "<li>%s</li>", items[i].name);
            if (li == NULL) {
                // 处理内存分配失败的情况
                free(html);
                free(items);
                return ESP_ERR_NO_MEM;
            }
            // 扩展 html 内存以容纳新的 <li> 元素
            char *temp = realloc(html, strlen(html) + strlen(li) + 1);
            if (temp == NULL) {
                // 处理内存分配失败的情况
                free(html);
                free(li);
                free(items);
                return ESP_ERR_NO_MEM;
            }
            html = temp;
            strcat(html, li);
            free(li);
        }
        // 结束 HTML 文档
        char *temp = realloc(html, strlen(html) + strlen("</ul>") + 1);
        if (temp == NULL) {
            // 处理内存分配失败的情况
            free(html);
            free(items);
            return ESP_ERR_NO_MEM;
        }
        html = temp;
        printf("\r\n");
        free(items);
        ESP_LOGI(TAG, "%s", html);
    } else {
        printf("Nothing saved yet!\r\n");
    }
    nvs_close(my_handle);
// index html
    char *indexBuffer;
    asiprintf(&indexBuffer,
              "<html><head><title>ESP32 IR Remote</title>"
              "</head><body>"
              "<h1>ESP32 IR Remote</h1>"
              "%s" // 插入新生成的 HTML 列表
              "<button onclick=\"sendRequest('/ir/send');\">IR Send</button>"
              "<br>"
              "<button id=\"send_btn\">IR Save</button>"
              "<label for=\"name_value\">send name</label><input id=\"name_value\" type=\"text\">"
              "<br>"
              "<button onclick=\"sendRequest('/ir/receive');\">IR Receive</button>"
              "<br>"
              "<button onclick=\"sendRequest('/ir/receive/cancel');\">IR Receive Cancel</button>"
              "</body>"
              " <script>"
              "function sendRequest(url) {"
              "let xhr = new XMLHttpRequest();"
              "xhr.open('GET', url, true);"
              "xhr.send();"
              "}"
              "let inputElement = document.getElementById(\"name_value\");"
              "let buttonElement = document.getElementById(\"send_btn\");"
              "buttonElement.addEventListener(\"click\", function () {"
              "let inputValue = inputElement.value;"
              "console.log(\"Input Value:\", inputValue);"
              "sendRequest('/ir/save?name=' + inputValue);"
              "});"
              "</script>"
              "</html>", html);

    httpd_resp_send(req, indexBuffer, strlen(indexBuffer));
    return ESP_OK;
}
