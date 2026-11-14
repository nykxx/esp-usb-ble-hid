// Bridge mode: any BLE HID device (keyboard, mouse, ...) -> the same HID device
// on USB. The USB report descriptor mirrors the device's Report Map, so every
// input report is forwarded as is, with its report id (see hid_descriptor.hpp).
//
// Optionally (CONFIG_BRIDGE_HID_PASSTHROUGH_BOOT_KEYBOARD) a boot keyboard is
// presented as well, for BIOS / UEFI hosts. It stays silent unless the host
// selects the boot protocol (SET_PROTOCOL(BOOT), which an operating system
// does not do); then the device's keyboard reports are rebuilt in the boot
// format and sent there instead, so keys are never delivered twice.
#include "sdkconfig.h"
#if CONFIG_BRIDGE_MODE_HID_PASSTHROUGH

#include "bridge.hpp"

#include <chrono>
#include <condition_variable>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "esp_system.h"

#include "logger.hpp"
#include "task.hpp"

#include "hid_descriptor.hpp"
#include "status_led.hpp"

#if HAS_DISPLAY
#include "gui_icons.hpp"
#endif

using namespace std::chrono_literals;

static espp::Logger logger({.tag = "HID Passthrough", .level = espp::Logger::Verbosity::INFO});

// whether the USB descriptor (the mirrored Report Map) declares report ids;
// without them reports go out with id 0 (no id byte on the wire)
static bool uses_report_ids = true;

// Restarts in a row allowed to switch the USB descriptor to a newly paired
// device's Report Map (guards against a device returning a varying map).
static constexpr uint32_t kMaxDescriptorRestarts = 2;

static uint8_t usb_report_id(uint8_t ble_report_id) { return uses_report_ids ? ble_report_id : 0; }

// where the keys are in the device's keyboard report (from its Report Map),
// for the boot keyboard; nullopt without a boot keyboard or a keyboard report
static std::optional<HidKeyboardLayout> keyboard_layout;

/********* BLE -> USB: input reports ***************/

// BLE notifications arrive on the NimBLE host task; they are queued and sent
// from one sender task, in order (a key press must never overtake its
// release), waiting for the interrupt IN endpoint to drain between reports.
struct InReport {
  bool boot{false}; // for the boot keyboard (no report id) instead of the HID function
  uint8_t id{0};
  std::vector<uint8_t> data;
};
static std::deque<InReport> hid_tx_queue;
static std::mutex hid_tx_mutex;
static std::condition_variable hid_tx_cv;
static std::unique_ptr<espp::Task> hid_sender_task;
static constexpr size_t kMaxQueuedReports = 64;
static constexpr auto kSendTimeout = 50ms; // give up on one report after this

static bool hid_sender_fn(std::mutex &, std::condition_variable &) {
  InReport rep;
  {
    std::unique_lock<std::mutex> lock(hid_tx_mutex);
    hid_tx_cv.wait_for(lock, 100ms, [] { return !hid_tx_queue.empty(); });
    if (hid_tx_queue.empty())
      return false;
    rep = std::move(hid_tx_queue.front());
    hid_tx_queue.pop_front();
  }
  if (!usb_is_mounted())
    return false; // nobody to send to
  const auto deadline = std::chrono::steady_clock::now() + kSendTimeout;
  std::error_code ec;
  auto write = [&]() {
    return rep.boot ? usb_write_boot_hid_report(rep.data, ec)
                    : usb_write_hid_report(rep.id, rep.data, ec);
  };
  while (!write()) {
    if (ec != std::errc::resource_unavailable_try_again ||
        std::chrono::steady_clock::now() >= deadline) {
      logger.warn_rate_limited("dropped HID report id {} ({})", rep.id, ec.message());
      return false;
    }
    std::this_thread::sleep_for(1ms);
  }
  return false; // don't stop the task
}

static void enqueue_hid(uint8_t report_id, std::span<const uint8_t> report, bool boot = false) {
  if (!usb_is_mounted())
    return;
  {
    std::lock_guard<std::mutex> lock(hid_tx_mutex);
    if (hid_tx_queue.size() >= kMaxQueuedReports) {
      logger.warn_rate_limited("HID queue full; dropping report id {}", report_id);
      return;
    }
    hid_tx_queue.push_back({boot, report_id, std::vector<uint8_t>(report.begin(), report.end())});
  }
  hid_tx_cv.notify_one();
}

// Length of the last report forwarded per report id, so every report can be
// neutralized (all keys / buttons released) when the device goes away.
static std::mutex sent_lengths_mutex;
static std::map<uint8_t, size_t> sent_lengths;

/********* USB -> BLE: output reports (keyboard LEDs) ***************/

// Received on the TinyUSB task, written to the device from this worker. Only
// the latest value per report id matters (LED state), so newer ones replace
// pending ones.
static std::mutex output_mutex;
static std::condition_variable output_cv;
struct OutputReport {
  uint32_t generation;
  std::vector<uint8_t> data;
};
static std::map<uint8_t, OutputReport> pending_output;
static std::unique_ptr<espp::Task> output_task;

static bool output_task_fn(std::mutex &, std::condition_variable &) {
  std::map<uint8_t, OutputReport> todo;
  {
    std::unique_lock<std::mutex> lk(output_mutex);
    output_cv.wait_for(lk, 100ms, [] { return !pending_output.empty(); });
    todo.swap(pending_output);
  }
  for (const auto &[id, report] : todo)
    ble_write_output_report(id, report.data, report.generation);
  return false; // don't stop the task
}

/********* Report Map -> USB descriptor ***************/

// The device's Report Map arrived (BLE scan-timer task): enumerate with it.
static bool on_report_map(const std::vector<uint8_t> &map) {
  if (map.empty()) {
    logger.warn("device has no readable Report Map; input forwarding remains disabled");
    return false;
  }
  if (hid_descriptor_matches(map)) {
    hid_descriptor_reset_restart_count(); // USB already matches the device
    return true;
  }
  if (hid_descriptor_restart_count() >= kMaxDescriptorRestarts) {
    logger.error("Report Map changed again after {} restarts; not restarting",
                 hid_descriptor_restart_count());
    return false;
  }
  if (!hid_descriptor_store(map))
    return false; // do not forward a mismatched report layout after an NVS error
  hid_descriptor_note_restart();
  logger.info("new Report Map: restarting to enumerate with it");
  std::this_thread::sleep_for(200ms); // let the log line out
  esp_restart();
  return false;
}

/********* bridge.hpp ***************/

void bridge_init() {
  ble_set_report_map_callback(on_report_map);

  hid_sender_task = espp::Task::make_unique(
      {.callback = hid_sender_fn,
       .task_config = {.name = "usb_hid_tx", .stack_size_bytes = 4096, .priority = 10}});
  hid_sender_task->start();
  output_task = espp::Task::make_unique(
      {.callback = output_task_fn, .task_config = {.name = "hid_out", .stack_size_bytes = 4096}});
  output_task->start();
}

UsbHidConfig bridge_usb_config() {
  UsbHidConfig cfg;
  cfg.vid = CONFIG_DONGLE_USB_VID;
  cfg.pid = CONFIG_DONGLE_USB_PID;
  cfg.manufacturer = CONFIG_DONGLE_USB_MANUFACTURER;
  cfg.product = CONFIG_DONGLE_USB_PRODUCT;

  // the descriptor of the last paired device (or a plain keyboard)
  cfg.hid.interface_name = "BLE HID Passthrough";
  cfg.hid.report_descriptor = hid_descriptor_load();
  uses_report_ids = hid_descriptor_uses_report_ids(cfg.hid.report_descriptor);
  cfg.hid.has_out_endpoint = true; // keyboard LED output reports
  cfg.hid.poll_interval_ms = 1;    // full-speed minimum: lowest added latency
  cfg.hid.on_receive = [](std::span<const uint8_t> data) {
    // TinyUSB task context: hand it over to the output worker (don't block)
    if (data.empty())
      return;
    uint8_t id = 0;
    if (uses_report_ids) {
      id = data[0];
      data = data.subspan(1);
    }
    {
      std::lock_guard<std::mutex> lk(output_mutex);
      pending_output[id] = {ble_link_generation(), {data.begin(), data.end()}};
    }
    output_cv.notify_one();
  };
#if CONFIG_BRIDGE_HID_PASSTHROUGH_BOOT_KEYBOARD
  keyboard_layout = hid_descriptor_keyboard_layout(cfg.hid.report_descriptor);
  if (keyboard_layout) {
    logger.info("boot keyboard: keyboard report id {}", keyboard_layout->report_id);
  } else {
    logger.warn("boot keyboard: no keyboard report in the descriptor; boot keys disabled");
  }
  espp::UsbDevice::HidFunction boot;
  boot.interface_name = "BLE HID Boot Keyboard";
  const auto boot_descriptor = hid_boot_keyboard_descriptor();
  boot.report_descriptor.assign(boot_descriptor.begin(), boot_descriptor.end());
  boot.boot_protocol = 1; // keyboard
  boot.poll_interval_ms = 8;
  boot.on_receive = [](std::span<const uint8_t> data) {
    // the BIOS' LED state (no report id): to the device's keyboard output report
    if (data.empty() || !keyboard_layout)
      return;
    {
      std::lock_guard<std::mutex> lk(output_mutex);
      pending_output[keyboard_layout->report_id] = {ble_link_generation(),
                                                    {data.begin(), data.end()}};
    }
    output_cv.notify_one();
  };
  cfg.boot_hid = boot;
#endif // CONFIG_BRIDGE_HID_PASSTHROUGH_BOOT_KEYBOARD
  cfg.on_unmount = []() {
    std::lock_guard<std::mutex> lock(hid_tx_mutex);
    hid_tx_queue.clear();
  };
  return cfg;
}

void bridge_on_ble_report(NimBLERemoteCharacteristic *chr, const uint8_t *data, size_t length) {
  if (!usb_is_mounted())
    return;
  // its report id comes from the characteristic's Report Reference
  const uint8_t ble_id = ble_report_id_for(chr).value_or(1);
  const std::span<const uint8_t> report(data, length);
  if (keyboard_layout && ble_id == keyboard_layout->report_id && usb_boot_protocol_active()) {
    // a BIOS / UEFI host: the keyboard goes to the boot keyboard only
    const auto boot = hid_boot_keyboard_report(*keyboard_layout, report);
    enqueue_hid(0, boot, true);
    return;
  }
  const uint8_t id = usb_report_id(ble_id);
  {
    std::lock_guard<std::mutex> lk(sent_lengths_mutex);
    sent_lengths[id] = length;
  }
  enqueue_hid(id, report);
}

void bridge_on_ble_battery(uint8_t) {
  // nothing to forward: the battery level is shown by the console
}

void bridge_on_ble_disconnect() {
  // send an all-zero report for every report id seen, so no key or button
  // stays pressed on the host when the device goes away mid-press
  std::map<uint8_t, size_t> lengths;
  {
    std::lock_guard<std::mutex> lk(sent_lengths_mutex);
    lengths.swap(sent_lengths);
  }
  {
    std::lock_guard<std::mutex> lk(output_mutex);
    pending_output.clear();
  }
  {
    std::lock_guard<std::mutex> lock(hid_tx_mutex);
    // Release reports must not be dropped behind a full queue of old keys.
    // The single sender may finish its in-flight report before these releases.
    hid_tx_queue.clear();
    if (usb_is_mounted()) {
      for (const auto &[id, len] : lengths)
        hid_tx_queue.push_back({false, id, std::vector<uint8_t>(len, 0)});
      if (keyboard_layout && usb_boot_protocol_active())
        hid_tx_queue.push_back({true, 0, std::vector<uint8_t>(8, 0)});
    }
  }
  hid_tx_cv.notify_one();
}

bool bridge_hid_ready() { return usb_is_mounted(); }

void bridge_poll() {}

std::string bridge_device_label() { return ble_connected_name(); }

#if HAS_DISPLAY
// GAP Appearance values (Bluetooth Assigned Numbers, HID category)
static constexpr uint16_t kAppearanceMouse = 0x03C2;

// A ZMK keyboard keeps ZMK's Device Information defaults unless its config
// overrides them: Manufacturer "ZMK Project", PnP ID 1D50:615E.
static bool is_zmk(const BleDeviceInfo &info) {
  return info.manufacturer == "ZMK Project" ||
         (info.vendor_id == 0x1D50 && info.product_id == 0x615E);
}

void bridge_update_display(Gui &gui) {
  static bool layout_set = false;
  if (!layout_set) {
    // [device icon + BLE] <-> [computer + USB], the device's name below the arrow
    gui.set_host_icon(&icon_computer);
    gui.set_label_below_arrow(true);
    layout_set = true;
  }
  const auto info = ble_connected_device_info();
  const lv_image_dsc_t *icon = &icon_keyboard;
  if (info.appearance == kAppearanceMouse)
    icon = &icon_mouse;
#if CONFIG_BRIDGE_HID_PASSTHROUGH_ZMK_LOGO
  if (is_zmk(info))
    icon = &icon_zmk_logo;
#endif
  static const lv_image_dsc_t *shown = nullptr;
  if (icon != shown) {
    gui.set_device_icon(icon);
    shown = icon;
  }
}
#endif // HAS_DISPLAY

#endif // CONFIG_BRIDGE_MODE_HID_PASSTHROUGH
