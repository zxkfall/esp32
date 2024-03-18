#include <stdio.h>
#include <driver/gpio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"

#define EXAMPLE_IR_RESOLUTION_HZ     (1*1000*1000) // 1MHz resolution, 1 tick = 1us
#define EXAMPLE_IR_TX_GPIO_NUM       18
#define EXAMPLE_IR_RX_GPIO_NUM       22
static const char *TAG = "WIFI_CONTROL";
static uint8_t led_level = 0x0;
static rmt_channel_handle_t tx_channel = NULL;
static rmt_channel_handle_t rx_channel = NULL;

static void configure_led(void) {
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(GPIO_NUM_16);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_NUM_16, GPIO_MODE_OUTPUT);
}

static bool
rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {

    BaseType_t high_task_wakeup = pdFALSE;
    //set led on
    led_level = ~led_level;
    gpio_set_level(GPIO_NUM_16, led_level);

    QueueHandle_t receive_queue = (QueueHandle_t) user_data;
    // 将接收到的 RMT 符号发送到解析任务的消息队列中
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    // 返回是否唤醒了任何任务

    return high_task_wakeup == pdTRUE;
}

void configure_ir_tx();

void configure_ir_rx();

void app_main() {
    configure_led();
    configure_ir_tx();
    configure_ir_rx();
    // enable channel
    ESP_ERROR_CHECK(rmt_enable(tx_channel));
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

    rmt_transmit_config_t transmit_config = {
            .loop_count = 0, // 不循环
    };

    // 准备开始接收
    ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
    // 等待 RX 完成信号
    rmt_rx_done_event_data_t rx_data;

    rmt_copy_encoder_config_t raw_config = {};
    rmt_encoder_handle_t raw_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&raw_config, &raw_encoder));
    int learning_times = 4;
    int current_learning_times = 0;
    size_t symbol_num = 300;
    rmt_symbol_word_t total_raw_symbols[learning_times][symbol_num];
    rmt_symbol_word_t *total_avg;
    uint8_t is_ready_send = 0;
    uint8_t is_inited = 0;
    ESP_LOGI(TAG, "Starting init params: symbol_length, please send first IR signal.\r\n");
    ESP_LOGI(TAG, "===============================================================\r\n");
    while (1) {
        if (!is_inited && xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(1000)) == pdPASS) {
            ESP_LOGI(TAG, "Set symbol_length successful, symbol_length: %d\r\n", rx_data.num_symbols);
            ESP_LOGI(TAG, "Next, you need to send `same` IR signal 4 times\r\n");
            ESP_LOGI(TAG, "===============================================================\r\n");
            is_inited = 1;
            ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
        } else {
            if (current_learning_times < learning_times && !is_ready_send &&
                xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(1000)) == pdPASS) {
                ESP_LOGI(TAG, "waiting for receive, start learning times: %d\r\n", current_learning_times + 1);
                rmt_symbol_word_t *receivedSymbols = rx_data.received_symbols;
                symbol_num = rx_data.num_symbols;
                for (int i = 0; i < rx_data.num_symbols; ++i) {
                    total_raw_symbols[current_learning_times][i] = receivedSymbols[i];
                }
                printf("lengths: %d---\r\n", rx_data.num_symbols);
                for (int i = 0; i < rx_data.num_symbols; ++i) {
                    printf("%d,%d ", receivedSymbols[i].duration0, receivedSymbols[i].duration1);
                    if (i + 1 % 20 == 0) {
                        printf("\r\n");
                    }
                }
                printf("\r\n");
                current_learning_times++;
                ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
            }
            if (current_learning_times == learning_times && !is_ready_send) {
                printf("\r\n");
                printf("start to get the average value\r\n");

                // 定义相近值的类别数组
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
                }

                printf("average value: \r\n");
                for (int i = 0; i < symbol_num; ++i) {
                    printf("%d,%d ", total_avg[i].duration0, total_avg[i].duration1);
                    if (i + 1 % 20 == 0) {
                        printf("\r\n");
                    }
                }

                is_ready_send = 1;
            }

            if (is_ready_send) {
                ESP_LOGI(TAG, "start to send the average value\r\n");
                ESP_LOGI(TAG, "===============================================================\r\n");
                ESP_ERROR_CHECK(rmt_transmit(tx_channel, raw_encoder, total_avg, symbol_num * 4, &transmit_config));
                for (int i = 0; i < symbol_num; ++i) {
                    printf("%d,%d ", total_avg[i].duration0, total_avg[i].duration1);
                    if (i + 1 % 20 == 0) {
                        printf("\r\n");
                    }
                }
                ESP_LOGI(TAG, "send the average value done\r\n");
                ESP_LOGI(TAG, "===============================================================\r\n");
                esp_rom_delay_us(1000000);
            }
        }
    }
}

void configure_ir_rx() {
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

void configure_ir_tx() {
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
