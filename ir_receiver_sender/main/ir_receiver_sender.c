#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"

#define EXAMPLE_IR_RESOLUTION_HZ     (1*1000*1000) // 1MHz resolution, 1 tick = 1us
#define EXAMPLE_IR_TX_GPIO_NUM       18
#define EXAMPLE_IR_RX_GPIO_NUM       22
#define EXAMPLE_IR_NEC_DECODE_MARGIN 200     // Tolerance for parsing RMT symbols into bit stream

/**
 * @brief NEC timing spec
 */
#define NEC_LEADING_CODE_DURATION_0  9000
#define NEC_LEADING_CODE_DURATION_1  4500
#define NEC_PAYLOAD_ZERO_DURATION_0  560
#define NEC_PAYLOAD_ZERO_DURATION_1  560
#define NEC_PAYLOAD_ONE_DURATION_0   560
#define NEC_PAYLOAD_ONE_DURATION_1   1690
#define NEC_REPEAT_CODE_DURATION_0   9000
#define NEC_REPEAT_CODE_DURATION_1   2250

static const char *TAG = "example";


static uint8_t led_level = 0x0;

static void configure_led(void) {
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(GPIO_NUM_16);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_NUM_16, GPIO_MODE_OUTPUT);
}

static bool
example_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {

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

static void set_duration_0(rmt_symbol_word_t *raw_symbol, uint16_t base_value, uint16_t acc) {
    if (raw_symbol->duration0 > base_value - acc && raw_symbol->duration0 < base_value + acc) {
        raw_symbol->duration0 = base_value;
    }
    raw_symbol->level0 = 1;
}

static void set_duration_1(rmt_symbol_word_t *raw_symbol, uint16_t base_value, uint16_t acc) {
    if (raw_symbol->duration1 > base_value - acc && raw_symbol->duration1 < base_value + acc) {
        raw_symbol->duration1 = base_value;
    }
    raw_symbol->level1 = 0;
}

static void set_duration(rmt_symbol_word_t *raw_symbol, uint16_t base_value, uint16_t acc_0, uint16_t acc_1) {
    set_duration_0(raw_symbol, base_value, acc_0);
    set_duration_1(raw_symbol, base_value, acc_1);
}

static void set_duration_with_same_acc(rmt_symbol_word_t *raw_symbol, uint16_t base_value, uint16_t acc) {
    set_duration_0(raw_symbol, base_value, acc);
    set_duration_1(raw_symbol, base_value, acc);
}

void app_main() {
    configure_led();
    //TX
    rmt_channel_handle_t tx_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,   // 选择时钟源
            .gpio_num = EXAMPLE_IR_TX_GPIO_NUM,                    // GPIO 编号
            .mem_block_symbols = 256,          // 内存块大小，即 256 * 4 = 1024 字节
            .resolution_hz = EXAMPLE_IR_RESOLUTION_HZ, // 1 MHz 滴答分辨率，即 1 滴答 = 1 µs
            .trans_queue_depth = 4,           // 设置后台等待处理的事务数量
            .flags.invert_out = false,        // 不反转输出信号
            .flags.with_dma = false,          // 不需要 DMA 后端
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_chan));
    //RX
    rmt_channel_handle_t rx_chan = NULL;
    rmt_rx_channel_config_t rx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,   // 选择时钟源
            .resolution_hz = 1 * 1000 * 1000, // 1 MHz 滴答分辨率，即 1 滴答 = 1 µs
            .mem_block_symbols = 256,          // 内存块大小，即 64 * 4 = 256 字节
            .gpio_num = EXAMPLE_IR_RX_GPIO_NUM,                    // GPIO 编号
            .flags.invert_in = false,         // 不反转输入信号
            .flags.with_dma = false,          // 不需要 DMA 后端
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_chan));

    rmt_carrier_config_t tx_carrier_cfg = {
            .duty_cycle = 0.33,                 // 载波占空比为 33%
            .frequency_hz = 38000,              // 38 KHz
            .flags.polarity_active_low = false, // 载波应调制到高电平
    };
    // 将载波调制到 TX 通道
    ESP_ERROR_CHECK(rmt_apply_carrier(tx_chan, &tx_carrier_cfg));

    // enable channel
    ESP_ERROR_CHECK(rmt_enable(tx_chan));
    ESP_ERROR_CHECK(rmt_enable(rx_chan));

    QueueHandle_t receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_event_callbacks_t cbs = {
            .on_recv_done = example_rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &cbs, receive_queue));

    // 以下时间要求均基于 NEC 协议
    rmt_receive_config_t receive_config = {
            .signal_range_min_ns = 1250,     // NEC 信号的最短持续时间为 560 µs，由于 1250 ns < 560 µs，有效信号不会视为噪声
            .signal_range_max_ns = 50 * 1000 * 1000, // NEC 信号的最长持续时间为 9000 µs，由于 12000000 ns > 9000 µs，接收不会提前停止
            .flags.en_partial_rx = false,    // 不需要部分接收
    };

    rmt_symbol_word_t raw_symbols[256]; // 64 个符号应足够存储一个标准 NEC 帧的数据

    rmt_transmit_config_t transmit_config = {
            .loop_count = 0, // 不循环
    };

    // 准备开始接收
    ESP_ERROR_CHECK(rmt_receive(rx_chan, raw_symbols, sizeof(raw_symbols), &receive_config));
    // 等待 RX 完成信号
    rmt_rx_done_event_data_t rx_data;

    rmt_copy_encoder_config_t raw_config = {};
    rmt_encoder_handle_t raw_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&raw_config, &raw_encoder));

    while (1) {
        if (xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(1000)) == pdPASS) {
            // 解析接收到的符号数据
            rmt_symbol_word_t *receivedSymbols = rx_data.received_symbols;
            printf("lengths: %d---\r\n", rx_data.num_symbols);

            if (rx_data.num_symbols >= 70) {
                ESP_LOGI(TAG, "RECEIVE");
                rmt_symbol_word_t raw_symbols_t[rx_data.num_symbols];
                for (int i = 0; i < rx_data.num_symbols; ++i) {
                    printf("%d,%d ", receivedSymbols[i].duration0, receivedSymbols[i].duration1);
                    if (i + 1 % 20 == 0) {
                        printf("\r\n");
                    }
                    if (receivedSymbols[i].duration1 > 4000) {
                        printf(" index:%d \r\n", i);
                    }
                }
                printf("\r\n");
                printf("raw_symbols_t.length: %d\r\n", sizeof(raw_symbols_t));
                for (int i = 0; i < rx_data.num_symbols; ++i) {
                    rmt_symbol_word_t *symbol = &raw_symbols_t[i];
                    rmt_symbol_word_t re_sy = receivedSymbols[i];
                    symbol->duration0 = re_sy.duration0;
                    symbol->duration1 = re_sy.duration1;
                    // Why use this? Because duration will deviate from the standard value, 
                    // its value will become smaller or larger when loop too many times
                    uint16_t allow_acc = 100;
                    set_duration_with_same_acc(symbol, 9000, allow_acc);
                    set_duration_with_same_acc(symbol, 4500, allow_acc);
                    set_duration_with_same_acc(symbol, 646, allow_acc);
                    set_duration_with_same_acc(symbol, 516, allow_acc);
                    set_duration_with_same_acc(symbol, 1643, allow_acc);
                    set_duration_with_same_acc(symbol, 20000, allow_acc);
                    set_duration_with_same_acc(symbol, 7250, allow_acc);
                }
                // also work fine if only learning once
//                for (int i = 0; i < rx_data.num_symbols; ++i) {
//                    rmt_symbol_word_t re_sy = receivedSymbols[i];
//                    raw_symbols_t[i].duration0 = re_sy.duration0;
//                    raw_symbols_t[i].duration1 = re_sy.duration1;
//                    raw_symbols_t[i].level0 = 1;
//                    raw_symbols_t[i].level1 = 0;
//                }

                ESP_LOGI(TAG, ">70\r\n");
                ESP_ERROR_CHECK(rmt_receive(rx_chan, raw_symbols, sizeof(raw_symbols), &receive_config));
                esp_rom_delay_us(1000000);
                ESP_LOGI(TAG, ">transmit\r\n");
                ESP_LOGI(TAG, ">transmit length: %d\r\n", sizeof(raw_symbols_t));
                ESP_ERROR_CHECK(
                        rmt_transmit(tx_chan, raw_encoder, raw_symbols_t, sizeof(raw_symbols_t), &transmit_config));
            } else {
                ESP_LOGI(TAG, "<70\r\n");
                ESP_ERROR_CHECK(rmt_receive(rx_chan, raw_symbols, sizeof(raw_symbols), &receive_config));
            }
        } else {}
    }
}