// Only used by the HID passthrough bridge (bridge_hid_passthrough.cpp).
#include "sdkconfig.h"
#if CONFIG_BRIDGE_MODE_HID_PASSTHROUGH

#include "hid_descriptor.hpp"

#include <algorithm>
#include <mutex>

#include "esp_attr.h"
#include "nvs.h"

#include "logger.hpp"

static espp::Logger logger({.tag = "HID Descriptor", .level = espp::Logger::Verbosity::INFO});

static constexpr const char *kNamespace = "hid_desc";
static constexpr const char *kKey = "report_map";
static constexpr size_t kMaxReportMap = 512; // HOGP limit for the Report Map value

// Keyboard (report id 1): modifiers, reserved byte, 6 key slots; LED output.
// Same layout as a ZMK keyboard report in its default (HKRO, 6 keys) config.
static const uint8_t kDefaultDescriptor[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x06,       // Usage (Keyboard)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,       //   Report ID (1)
    0x05, 0x07,       //   Usage Page (Keyboard/Keypad)
    0x19, 0xE0,       //   Usage Minimum (Left Control)
    0x29, 0xE7,       //   Usage Maximum (Right GUI)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x08,       //   Report Count (8)
    0x81, 0x02,       //   Input (Data, Var, Abs)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x01,       //   Input (Const)
    0x05, 0x08,       //   Usage Page (LEDs)
    0x19, 0x01,       //   Usage Minimum (Num Lock)
    0x29, 0x05,       //   Usage Maximum (Kana)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x05,       //   Report Count (5)
    0x91, 0x02,       //   Output (Data, Var, Abs)
    0x75, 0x03,       //   Report Size (3)
    0x95, 0x01,       //   Report Count (1)
    0x91, 0x01,       //   Output (Const)
    0x05, 0x07,       //   Usage Page (Keyboard/Keypad)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x19, 0x00,       //   Usage Minimum (0)
    0x29, 0xFF,       //   Usage Maximum (255)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x06,       //   Report Count (6)
    0x81, 0x00,       //   Input (Data, Array, Abs)
    0xC0,             // End Collection
};

static std::mutex mutex;
static std::vector<uint8_t> in_use; // what start_usb() was given this boot
static bool cached = false;

// Survives esp_restart() (not a power cycle): counts descriptor restarts.
RTC_NOINIT_ATTR static uint32_t restart_magic;
RTC_NOINIT_ATTR static uint32_t restart_count;
static constexpr uint32_t kRestartMagic = 0x48494444; // "HIDD"

static bool read_cached(std::vector<uint8_t> &out) {
  nvs_handle_t h;
  if (nvs_open(kNamespace, NVS_READONLY, &h) != ESP_OK)
    return false;
  size_t len = 0;
  bool ok = nvs_get_blob(h, kKey, nullptr, &len) == ESP_OK && len > 0 && len <= kMaxReportMap;
  if (ok) {
    out.resize(len);
    ok = nvs_get_blob(h, kKey, out.data(), &len) == ESP_OK;
  }
  nvs_close(h);
  return ok;
}

std::vector<uint8_t> hid_descriptor_load() {
  std::lock_guard<std::mutex> lk(mutex);
  std::vector<uint8_t> map;
  if (read_cached(map)) {
    logger.info("using the cached Report Map ({} bytes)", map.size());
    cached = true;
    in_use = map;
  } else {
    logger.info("no cached Report Map: using the default keyboard descriptor");
    cached = false;
    in_use.assign(std::begin(kDefaultDescriptor), std::end(kDefaultDescriptor));
  }
  return in_use;
}

bool hid_descriptor_is_cached() {
  std::lock_guard<std::mutex> lk(mutex);
  return cached;
}

bool hid_descriptor_store(std::span<const uint8_t> report_map) {
  if (report_map.empty() || report_map.size() > kMaxReportMap) {
    logger.warn("ignoring a Report Map of {} bytes", report_map.size());
    return false;
  }
  std::lock_guard<std::mutex> lk(mutex);
  if (std::equal(report_map.begin(), report_map.end(), in_use.begin(), in_use.end()))
    return false; // already enumerated with it
  nvs_handle_t h;
  esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &h);
  if (err == ESP_OK) {
    err = nvs_set_blob(h, kKey, report_map.data(), report_map.size());
    if (err == ESP_OK)
      err = nvs_commit(h);
    nvs_close(h);
  }
  if (err != ESP_OK) {
    logger.error("could not store the Report Map: {}", esp_err_to_name(err));
    return false;
  }
  logger.info("stored a new Report Map ({} bytes)", report_map.size());
  return true;
}

void hid_descriptor_clear() {
  nvs_handle_t h;
  if (nvs_open(kNamespace, NVS_READWRITE, &h) != ESP_OK)
    return;
  nvs_erase_key(h, kKey);
  nvs_commit(h);
  nvs_close(h);
}

bool hid_descriptor_uses_report_ids(std::span<const uint8_t> d) {
  for (size_t i = 0; i < d.size();) {
    const uint8_t prefix = d[i];
    if (prefix == 0xFE) { // long item: [0xFE][size][tag][data...]
      if (i + 1 >= d.size())
        break;
      i += 3 + d[i + 1];
      continue;
    }
    const size_t size = (prefix & 0x03) == 3 ? 4 : (prefix & 0x03);
    if ((prefix & 0xFC) == 0x84) // global item, tag Report ID
      return true;
    i += 1 + size;
  }
  return false;
}

uint32_t hid_descriptor_restart_count() {
  return restart_magic == kRestartMagic ? restart_count : 0;
}

void hid_descriptor_note_restart() {
  if (restart_magic != kRestartMagic) {
    restart_magic = kRestartMagic;
    restart_count = 0;
  }
  ++restart_count;
}

void hid_descriptor_reset_restart_count() {
  restart_magic = kRestartMagic;
  restart_count = 0;
}

#endif // CONFIG_BRIDGE_MODE_HID_PASSTHROUGH
