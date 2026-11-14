// Exercise the real HID descriptor code with an in-memory NVS backend.
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

#include "hid_descriptor.hpp"
#include "nvs.h"

static std::vector<uint8_t> stored;
static bool fail_writes = false;
esp_err_t nvs_open(const char *, int mode, nvs_handle_t *handle) {
  *handle = 1;
  return mode == NVS_READWRITE && fail_writes ? -1 : ESP_OK;
}
esp_err_t nvs_get_blob(nvs_handle_t, const char *, void *data, size_t *size) {
  if (stored.empty())
    return -1;
  if (data) {
    if (*size < stored.size())
      return -1;
    std::memcpy(data, stored.data(), stored.size());
  }
  *size = stored.size();
  return ESP_OK;
}
esp_err_t nvs_set_blob(nvs_handle_t, const char *, const void *data, size_t size) {
  const auto *bytes = static_cast<const uint8_t *>(data);
  stored.assign(bytes, bytes + size);
  return ESP_OK;
}
esp_err_t nvs_commit(nvs_handle_t) { return ESP_OK; }
esp_err_t nvs_erase_key(nvs_handle_t, const char *) {
  stored.clear();
  return ESP_OK;
}
void nvs_close(nvs_handle_t) {}
const char *esp_err_to_name(esp_err_t) { return "simulated NVS error"; }

int main() {
  const auto original = hid_descriptor_load();
  assert(!hid_descriptor_is_cached());
  assert(hid_descriptor_matches(original));
  assert(!hid_descriptor_matches({}));
  assert(!hid_descriptor_store(original));
  auto different = original;
  const auto id = std::find(different.begin(), different.end(), 0x85);
  assert(id != different.end());
  *(id + 1) = 2;
  fail_writes = true;
  assert(!hid_descriptor_store(different));
  assert(!hid_descriptor_matches(different)); // a save failure must never enable forwarding
  fail_writes = false;
  assert(hid_descriptor_store(different));
  assert(hid_descriptor_matches(original)); // current USB enumeration is still unchanged
  assert(!hid_descriptor_matches(different));
  assert(hid_descriptor_load() == different); // simulated reboot
  assert(hid_descriptor_matches(different) && hid_descriptor_is_cached());
  assert(!hid_descriptor_store(different)); // no reboot for the same layout
  assert(!hid_descriptor_store({}));
  assert(!hid_descriptor_store(std::vector<uint8_t>(513, 0)));

  const auto hkro = hid_descriptor_keyboard_layout(original);
  assert(hkro && hkro->report_id == 1 && hkro->keys_count == 6);
  const std::array<uint8_t, 8> keys{2, 0, 4, 5, 0, 0, 0, 0};
  assert(hid_boot_keyboard_report(*hkro, keys) == keys);
  const std::vector<uint8_t> nkro_map{0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x85, 0x01, 0x05,
                                      0x07, 0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00, 0x25, 0x01,
                                      0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x19, 0x00, 0x29,
                                      0x77, 0x95, 0x78, 0x81, 0x02, 0xc0};
  const auto nkro = hid_descriptor_keyboard_layout(nkro_map);
  assert(nkro && nkro->bitmap_count == 120);
  std::vector<uint8_t> bits(16, 0);
  bits[0] = 2;
  bits[1] = (1 << 4) | (1 << 5);
  assert(hid_boot_keyboard_report(*nkro, bits) == keys);
  bits[1] = 0xf0;
  bits[2] = 0x07; // seven keys: BIOS rollover
  const auto rollover = hid_boot_keyboard_report(*nkro, bits);
  assert(rollover[0] == 2);
  assert(std::all_of(rollover.begin() + 2, rollover.end(), [](auto c) { return c == 1; }));
  hid_descriptor_clear();
  assert(hid_descriptor_load() == original);
  std::puts("HID descriptor switching, NVS failure and boot keyboard tests passed");
}
