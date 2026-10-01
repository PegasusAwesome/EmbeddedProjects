#pragma once
#include <cstddef>
#include <cstdint>
#include "gpio.h"

using rmt_channel_handle_t = void*;
using rmt_encoder_handle_t = void*;
constexpr int RMT_CLK_SRC_DEFAULT = 0;
struct rmt_symbol_word_t {
    uint32_t duration0 : 15;
    uint32_t level0 : 1;
    uint32_t duration1 : 15;
    uint32_t level1 : 1;
};
struct rmt_tx_channel_config_t {
    gpio_num_t gpio_num;
    int clk_src;
    uint32_t resolution_hz;
    size_t mem_block_symbols;
    size_t trans_queue_depth;
};
struct rmt_copy_encoder_config_t {};
struct rmt_transmit_config_t {};
esp_err_t rmt_new_tx_channel(const rmt_tx_channel_config_t*, rmt_channel_handle_t*);
esp_err_t rmt_new_copy_encoder(const rmt_copy_encoder_config_t*, rmt_encoder_handle_t*);
esp_err_t rmt_tx_switch_gpio(rmt_channel_handle_t, gpio_num_t, bool);
esp_err_t rmt_enable(rmt_channel_handle_t);
esp_err_t rmt_transmit(rmt_channel_handle_t, rmt_encoder_handle_t, const void*, size_t, const rmt_transmit_config_t*);
esp_err_t rmt_tx_wait_all_done(rmt_channel_handle_t, int);
esp_err_t rmt_disable(rmt_channel_handle_t);
