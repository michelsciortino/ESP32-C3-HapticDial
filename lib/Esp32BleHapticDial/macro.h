#ifndef HEADER_MACRO
#define HEADER_MACRO
#include "esp_log.h"

#define INFO(format) ESP_LOGI(LOG_TAG, format)
#define INFO(format, __VA_ARGS__...) ESP_LOGI(LOG_TAG, format, ##__VA_ARGS__)
#endif