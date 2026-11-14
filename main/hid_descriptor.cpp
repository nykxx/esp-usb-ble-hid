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

bool hid_descriptor_matches(std::span<const uint8_t> report_map) {
  std::lock_guard<std::mutex> lk(mutex);
  return !report_map.empty() &&
         std::equal(report_map.begin(), report_map.end(), in_use.begin(), in_use.end());
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

// --- boot keyboard (BIOS / UEFI) ---------------------------------------------------

static const uint8_t kBootKeyboardDescriptor[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x06,       // Usage (Keyboard)
    0xA1, 0x01,       // Collection (Application)
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

std::span<const uint8_t> hid_boot_keyboard_descriptor() { return kBootKeyboardDescriptor; }

static constexpr uint16_t kUsagePageKeyboard = 0x07;
static constexpr uint16_t kUsageLeftControl = 0xE0;
static constexpr uint16_t kUsageRightGui = 0xE7;
static constexpr uint8_t kErrorRollOver = 0x01;

std::optional<HidKeyboardLayout> hid_descriptor_keyboard_layout(std::span<const uint8_t> d) {
  // Only what locating the keyboard fields needs: global state (with
  // Push / Pop), the local usages of the next main item, and the running bit
  // offset of every input report.
  struct Globals {
    uint16_t usage_page{0};
    uint32_t report_size{0};
    uint32_t report_count{0};
    uint8_t report_id{0};
  };
  Globals g;
  std::vector<Globals> stack;
  uint32_t usage_min = 0, usage_max = 0;
  bool have_range = false;
  std::array<uint32_t, 256> input_bits{}; // per report id
  HidKeyboardLayout layout;
  bool found = false;

  for (size_t i = 0; i < d.size();) {
    const uint8_t prefix = d[i];
    if (prefix == 0xFE) { // long item: [0xFE][size][tag][data...]
      if (i + 1 >= d.size())
        break;
      i += 3 + d[i + 1];
      continue;
    }
    const size_t size = (prefix & 0x03) == 3 ? 4 : (prefix & 0x03);
    if (i + 1 + size > d.size())
      break;
    uint32_t value = 0;
    for (size_t b = 0; b < size; ++b)
      value |= static_cast<uint32_t>(d[i + 1 + b]) << (8 * b);
    const uint8_t type = (prefix >> 2) & 0x03;
    const uint8_t tag = prefix >> 4;
    i += 1 + size;

    if (type == 1) { // global
      switch (tag) {
      case 0x0:
        g.usage_page = static_cast<uint16_t>(value);
        break;
      case 0x7:
        g.report_size = value;
        break;
      case 0x8:
        g.report_id = static_cast<uint8_t>(value);
        break;
      case 0x9:
        g.report_count = value;
        break;
      case 0xA:
        stack.push_back(g);
        break;
      case 0xB:
        if (!stack.empty()) {
          g = stack.back();
          stack.pop_back();
        }
        break;
      default:
        break;
      }
    } else if (type == 2) { // local
      if (tag == 0x1) {
        usage_min = value;
        have_range = true;
      } else if (tag == 0x2) {
        usage_max = value;
      }
    } else if (type == 0) { // main
      if (tag == 0x8) {     // Input
        uint32_t &bits = input_bits[g.report_id];
        const bool constant = value & 0x01;
        const bool variable = value & 0x02;
        const bool same_report = !found || g.report_id == layout.report_id;
        if (!constant && g.usage_page == kUsagePageKeyboard && same_report) {
          if (variable && g.report_size == 1 && have_range && usage_min == kUsageLeftControl &&
              usage_max == kUsageRightGui && g.report_count == 8 && layout.modifiers_bit < 0) {
            layout.modifiers_bit = static_cast<int>(bits);
          } else if (!variable && g.report_size == 8 && layout.keys_bit < 0) {
            layout.keys_bit = static_cast<int>(bits);
            layout.keys_count = static_cast<uint8_t>(std::min<uint32_t>(g.report_count, 255));
          } else if (variable && g.report_size == 1 && have_range && usage_max > usage_min &&
                     layout.bitmap_bit < 0) {
            layout.bitmap_bit = static_cast<int>(bits);
            layout.bitmap_usage_min = static_cast<uint16_t>(usage_min);
            layout.bitmap_count = static_cast<uint16_t>(g.report_count);
          }
          if (!found) {
            found = true;
            layout.report_id = g.report_id;
          }
        }
        bits += g.report_size * g.report_count;
      }
      // every main item (and a collection) ends the local items
      usage_min = usage_max = 0;
      have_range = false;
    }
  }
  if (!found || (layout.keys_bit < 0 && layout.bitmap_bit < 0))
    return std::nullopt;
  return layout;
}

static bool bit_at(std::span<const uint8_t> data, uint32_t bit) {
  const uint32_t byte = bit / 8;
  return byte < data.size() && (data[byte] >> (bit % 8)) & 1;
}

static uint8_t byte_at(std::span<const uint8_t> data, uint32_t bit) {
  uint8_t value = 0;
  for (int b = 0; b < 8; ++b)
    value |= static_cast<uint8_t>(bit_at(data, bit + b)) << b;
  return value;
}

std::array<uint8_t, 8> hid_boot_keyboard_report(const HidKeyboardLayout &layout,
                                                std::span<const uint8_t> report) {
  std::array<uint8_t, 8> boot{};
  if (layout.modifiers_bit >= 0)
    boot[0] = byte_at(report, layout.modifiers_bit);
  size_t keys = 0;
  bool rollover = false;
  auto add_key = [&](uint16_t usage) {
    if (usage >= kUsageLeftControl && usage <= kUsageRightGui) {
      boot[0] |= static_cast<uint8_t>(1 << (usage - kUsageLeftControl));
    } else if (usage > kErrorRollOver && usage <= 0xFF) {
      if (keys < 6)
        boot[2 + keys++] = static_cast<uint8_t>(usage);
      else
        rollover = true;
    }
  };
  if (layout.keys_bit >= 0) {
    for (uint32_t k = 0; k < layout.keys_count; ++k)
      add_key(byte_at(report, layout.keys_bit + 8 * k));
  }
  if (layout.bitmap_bit >= 0) {
    for (uint32_t k = 0; k < layout.bitmap_count; ++k) {
      if (bit_at(report, layout.bitmap_bit + k))
        add_key(static_cast<uint16_t>(layout.bitmap_usage_min + k));
    }
  }
  if (rollover)
    std::fill(boot.begin() + 2, boot.end(), kErrorRollOver);
  return boot;
}

#endif // CONFIG_BRIDGE_MODE_HID_PASSTHROUGH
