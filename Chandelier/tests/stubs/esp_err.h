#pragma once
#include <stdexcept>
using esp_err_t = int;
constexpr int ESP_OK = 0;
#define ESP_ERROR_CHECK(expression) do { if ((expression) != ESP_OK) throw std::runtime_error(#expression); } while (false)
