#pragma once
#include "esp_err.h"
using gpio_num_t = int;
constexpr gpio_num_t GPIO_NUM_NC = -1;
constexpr int GPIO_MODE_OUTPUT = 1;
constexpr int GPIO_PULLDOWN_ONLY = 2;
esp_err_t gpio_set_pull_mode(gpio_num_t pin, int mode);
esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level);
esp_err_t gpio_set_direction(gpio_num_t pin, int mode);
