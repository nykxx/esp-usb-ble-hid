#pragma once
#include <cstddef>
#include <cstdint>

using nvs_handle_t = uint32_t;
using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
constexpr int NVS_READONLY = 0;
constexpr int NVS_READWRITE = 1;
esp_err_t nvs_open(const char *, int mode, nvs_handle_t *);
esp_err_t nvs_get_blob(nvs_handle_t, const char *, void *, size_t *);
esp_err_t nvs_set_blob(nvs_handle_t, const char *, const void *, size_t);
esp_err_t nvs_commit(nvs_handle_t);
esp_err_t nvs_erase_key(nvs_handle_t, const char *);
void nvs_close(nvs_handle_t);
const char *esp_err_to_name(esp_err_t);
