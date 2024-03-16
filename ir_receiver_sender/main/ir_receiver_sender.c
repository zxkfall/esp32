/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <driver/gpio.h>
#include <hal/rmt_ll.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"

#define EXAMPLE_IR_RESOLUTION_HZ     1000000 // 1MHz resolution, 1 tick = 1us
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

/**
 * @brief Saving NEC decode results
 */
static uint16_t s_nec_code_address;
static uint16_t s_nec_code_command;

/**
 * @brief Check whether a duration is within expected range
 */
static inline bool nec_check_in_range(uint32_t signal_duration, uint32_t spec_duration) {
    return (signal_duration < (spec_duration + EXAMPLE_IR_NEC_DECODE_MARGIN)) &&
           (signal_duration > (spec_duration - EXAMPLE_IR_NEC_DECODE_MARGIN));
}

/**
 * @brief Check whether a RMT symbol represents NEC logic zero
 */
static bool nec_parse_logic0(rmt_symbol_word_t *rmt_nec_symbols) {
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ZERO_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ZERO_DURATION_1);
}

/**
 * @brief Check whether a RMT symbol represents NEC logic one
 */
static bool nec_parse_logic1(rmt_symbol_word_t *rmt_nec_symbols) {
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ONE_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ONE_DURATION_1);
}

/**
 * @brief Decode RMT symbols into NEC address and command
 */
static bool nec_parse_frame(rmt_symbol_word_t *rmt_nec_symbols) {
    rmt_symbol_word_t *cur = rmt_nec_symbols;
    uint16_t address = 0;
    uint16_t command = 0;
    bool valid_leading_code = nec_check_in_range(cur->duration0, NEC_LEADING_CODE_DURATION_0) &&
                              nec_check_in_range(cur->duration1, NEC_LEADING_CODE_DURATION_1);
    if (!valid_leading_code) {
        return false;
    }
    cur++;
    for (int i = 0; i < 16; i++) {
        if (nec_parse_logic1(cur)) {
            address |= 1 << i;
        } else if (nec_parse_logic0(cur)) {
            address &= ~(1 << i);
        } else {
            return false;
        }
        cur++;
    }
    for (int i = 0; i < 16; i++) {
        if (nec_parse_logic1(cur)) {
            command |= 1 << i;
        } else if (nec_parse_logic0(cur)) {
            command &= ~(1 << i);
        } else {
            return false;
        }
        cur++;
    }
    // save address and command
    s_nec_code_address = address;
    s_nec_code_command = command;
    return true;
}

/**
 * @brief Check whether the RMT symbols represent NEC repeat code
 */
static bool nec_parse_frame_repeat(rmt_symbol_word_t *rmt_nec_symbols) {
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_REPEAT_CODE_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_REPEAT_CODE_DURATION_1);
}

/**
 * @brief Decode RMT symbols into NEC scan code and print the result
 */
static void example_parse_nec_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num) {
    printf("NEC frame start---\r\n");
    printf("lengths: %d---\r\n", symbol_num);
    for (size_t i = 0; i < symbol_num; i++) {
        printf("{%d:%d},{%d:%d}\r\n", rmt_nec_symbols[i].level0, rmt_nec_symbols[i].duration0,
               rmt_nec_symbols[i].level1, rmt_nec_symbols[i].duration1);
    }
    printf("---NEC frame end: ");
    // decode RMT symbols
    switch (symbol_num) {
        case 34: // NEC normal frame
            if (nec_parse_frame(rmt_nec_symbols)) {
                printf("Address=%04X, Command=%04X\r\n\r\n", s_nec_code_address, s_nec_code_command);
            }
            break;
        case 2: // NEC repeat frame
            if (nec_parse_frame_repeat(rmt_nec_symbols)) {
                printf("Address=%04X, Command=%04X, repeat\r\n\r\n", s_nec_code_address, s_nec_code_command);
            }
            break;
        default:
            printf("Unknown NEC frame\r\n\r\n");
            break;
    }
}


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


void app_main() {
    configure_led();
    //TX
    rmt_channel_handle_t tx_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,   // 选择时钟源
            .gpio_num = 18,                    // GPIO 编号
            .mem_block_symbols = 256,          // 内存块大小，即 64 * 4 = 256 字节
            .resolution_hz = 1 * 1000 * 1000, // 1 MHz 滴答分辨率，即 1 滴答 = 1 µs
            .trans_queue_depth = 4,           // 设置后台等待处理的事务数量
            .flags.invert_out = true,        // 不反转输出信号
            .flags.with_dma = false,          // 不需要 DMA 后端
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_chan));
    //RX
    rmt_channel_handle_t rx_chan = NULL;
    rmt_rx_channel_config_t rx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,   // 选择时钟源
            .resolution_hz = 1 * 1000 * 1000, // 1 MHz 滴答分辨率，即 1 滴答 = 1 µs
            .mem_block_symbols = 256,          // 内存块大小，即 64 * 4 = 256 字节
            .gpio_num = 22,                    // GPIO 编号
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

    rmt_carrier_config_t rx_carrier_cfg = {
            .duty_cycle = 0.33,                 // 载波占空比为 33%
            .frequency_hz = 25000,              // 载波频率为 25 KHz，应小于发射器的载波频率
            .flags.polarity_active_low = false, // 载波调制到高电平
    };
    // 从 RX 通道解调载波
//    ESP_ERROR_CHECK(rmt_apply_carrier(rx_chan, &rx_carrier_cfg));
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
            .signal_range_max_ns = 50*1000*1000, // NEC 信号的最长持续时间为 9000 µs，由于 12000000 ns > 9000 µs，接收不会提前停止
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
                rmt_symbol_word_t raw_symbols_t[rx_data.num_symbols/2];
                rmt_symbol_word_t raw_symbols_t_2[rx_data.num_symbols/2];
                for (int i = 0; i < rx_data.num_symbols; ++i) {
                    printf("%d,%d ", receivedSymbols[i].duration0, receivedSymbols[i].duration1);
                    if (i+1%20 == 0){
                        printf("\r\n");
                    }
//                    if (receivedSymbols[i].duration1 > 19000) {
//                        printf(" index:%d \r\n", i);
//                    }
                    if (receivedSymbols[i].duration1 > 4000) {
                        printf(" index:%d \r\n", i);
                    }
                }
                printf("\r\n");
                printf("raw_symbols_t.length: %d\r\n", sizeof(raw_symbols_t));
//                for (int i = 0; i < rx_data.num_symbols; ++i) {
////                    uint16_t duration0 = receivedSymbols[i].duration0;
////                    raw_symbols_t[i].duration0 = duration0;
////                    uint16_t duration1 = receivedSymbols[i].duration1;
////                    raw_symbols_t[i].duration1 = duration1;
////                    if (duration0 < 9000 + 100 && duration0 > 9000 - 100) {
////                        raw_symbols_t[i].duration0 = 9000;
////                    }
////                    if (duration1 < 9000 + 100 && duration1 > 9000 - 100) {
////                        raw_symbols_t[i].duration1 = 9000;
////                    }
////                    if (duration0 < 4500 + 100 && duration0 > 4500 - 100) {
////                        raw_symbols_t[i].duration0 = 4500;
////                    }
////                    if (duration1 < 4500 + 100 && duration1 > 4500 - 100) {
////                        raw_symbols_t[i].duration1 = 4500;
////                    }
////                    if (duration0 < 646 + 100 && duration0 > 646 - 100) {
////                        raw_symbols_t[i].duration0 = 646;
////                    }
////                    if (duration1 < 646 + 100 && duration1 > 646 - 100) {
////                        raw_symbols_t[i].duration1 = 646;
////                    }
////                    if (duration0 < 516 + 100 && duration0 > 516 - 100) {
////                        raw_symbols_t[i].duration0 = 516;
////                    }
////                    if (duration1 < 516 + 100 && duration1 > 516 - 100) {
////                        raw_symbols_t[i].duration1 = 516;
////                    }
////                    if (duration0 < 1643 + 100 && duration0 > 1643 - 100) {
////                        raw_symbols_t[i].duration0 = 1643;
////                    }
////                    if (duration1 < 1643 + 100 && duration1 > 1643 - 100) {
////                        raw_symbols_t[i].duration1 = 1643;
////                    }
////                    if (duration0 < 20000 + 100 && duration0 > 20000 - 100) {
////                        raw_symbols_t[i].duration0 = 20000;
////                    }
////                    if (duration1 < 20000 + 100 && duration1 > 20000 - 100) {
////                        raw_symbols_t[i].duration1 = 20000;
////                    }
////                    if (duration0 == 0) {
////                        raw_symbols_t[i].duration0 = 20000;
////                    }
////                    if (duration1 == 0) {
////                        raw_symbols_t[i].duration1 = 20000;
////                    }
////                    raw_symbols_t[i].level0 = 1;
////                    raw_symbols_t[i].level1 = 0;
//
//
//                    rmt_symbol_word_t re_sy = receivedSymbols[i];
//                    raw_symbols_t[i].duration0 = re_sy.duration0;
//                    raw_symbols_t[i].duration1 = re_sy.duration1;
////                    if (i == 69){
////                        raw_symbols_t[i].duration1 = 1650;
////                    }
//                    raw_symbols_t[i].level0 = 1;
//                    raw_symbols_t[i].level1 = 0;
//                }

                for (int i = 0; i < 70; ++i) {
                    rmt_symbol_word_t re_sy = receivedSymbols[i];
                    raw_symbols_t[i].duration0 = re_sy.duration0;
                    raw_symbols_t[i].duration1 = re_sy.duration1;
                    raw_symbols_t[i].level0 = 1;
                    raw_symbols_t[i].level1 = 0;
                }

                for (int i = 0; i < 18; ++i) {
                    raw_symbols_t_2[i].duration0 = 1000;
                    raw_symbols_t_2[i].duration1 = 1000;
                    raw_symbols_t_2[i].level0 = 1;
                    raw_symbols_t_2[i].level1 = 1;
                }

                for (int i = 0; i < 70; ++i) {
                    rmt_symbol_word_t re_sy = receivedSymbols[i+70];
                    raw_symbols_t_2[i].duration0 = re_sy.duration0;
                    raw_symbols_t_2[i].duration1 = re_sy.duration1;
                    raw_symbols_t_2[i].level0 = 1;
                    raw_symbols_t_2[i].level1 = 0;
                }

                ESP_LOGI(TAG, ">70\r\n");
                ESP_ERROR_CHECK(rmt_receive(rx_chan, raw_symbols, sizeof(raw_symbols), &receive_config));
                esp_rom_delay_us(1000000);
                ESP_LOGI(TAG, ">transmit\r\n");
                ESP_ERROR_CHECK(rmt_transmit(tx_chan, raw_encoder, raw_symbols_t, sizeof(raw_symbols_t), &transmit_config));
                esp_rom_delay_us(40*1000);
                ESP_ERROR_CHECK(rmt_transmit(tx_chan, raw_encoder, raw_symbols_t_2, sizeof(raw_symbols_t_2), &transmit_config));
            } else {
                ESP_LOGI(TAG, "<70\r\n");
                ESP_ERROR_CHECK(rmt_receive(rx_chan, raw_symbols, sizeof(raw_symbols), &receive_config));
            }
        } else {}
    }
}