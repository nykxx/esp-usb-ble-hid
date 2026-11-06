#include "usb.hpp"

#include "tusb.h" // tud_connect / tud_disconnect (pull-up re-arm)

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include "esp_mac.h"

#include "format.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;

static espp::Logger logger({.tag = "USB", .level = espp::Logger::Verbosity::INFO});

static std::unique_ptr<espp::UsbDevice> usb;
static std::atomic<bool> mounted{false};
static std::atomic<uint32_t> hid_reports_sent{0}; // input reports accepted by TinyUSB
static std::function<void()> on_mount_callback;
static std::function<void()> on_unmount_callback;

#if CONFIG_DONGLE_USB_VENDOR_INTERFACE
// --- Vendor RX: espp::DispatcherWorker ------------------------------------------
//
// Bytes arrive on the TinyUSB task, where handlers must not block (an OTA
// BEGIN erases a partition, a core-dump erase takes tens of milliseconds). The
// worker queues them (bounded) and runs the module handlers -- and their
// replies -- on its own task; on overflow it resynchronizes the parser and
// tells the services so an in-flight transfer can be aborted.
static std::unique_ptr<espp::DispatcherWorker> vendor_link;
// set from the app task, invoked from the worker: guard it so it may be
// (re)set at any time without racing the worker
static std::mutex rx_overflow_callback_mutex;
static std::function<void()> rx_overflow_callback;

static void on_vendor_receive(std::span<const uint8_t> data) {
  // TinyUSB task context: just queue the bytes and wake the worker.
  usb_dispatcher().push(data);
}

espp::DispatcherWorker &usb_dispatcher() {
  if (!vendor_link) {
    vendor_link = std::make_unique<espp::DispatcherWorker>(espp::DispatcherWorker::Config{
        .send = [](std::span<const uint8_t> frame) { usb_write_vendor(frame); },
        .on_overflow =
            [] {
              std::function<void()> callback;
              {
                std::lock_guard<std::mutex> lock(rx_overflow_callback_mutex);
                callback = rx_overflow_callback;
              }
              if (callback)
                callback(); // on the worker: it may send frames / block
            },
        .task_config = {.name = "usb_rx", .stack_size_bytes = 8192},
        .log_level = espp::Logger::Verbosity::INFO});
  }
  return *vendor_link;
}

void usb_set_rx_overflow_callback(std::function<void()> callback) {
  std::lock_guard<std::mutex> lock(rx_overflow_callback_mutex);
  rx_overflow_callback = std::move(callback);
}
#endif // CONFIG_DONGLE_USB_VENDOR_INTERFACE

// --- public API ------------------------------------------------------------------

bool usb_is_mounted() { return mounted.load(); }

uint32_t usb_hid_reports_sent() { return hid_reports_sent.load(); }

bool usb_write_hid_report(uint8_t report_id, std::span<const uint8_t> report, std::error_code &ec) {
  if (!usb) {
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  if (!usb->write_hid_report(report_id, report, ec))
    return false;
  hid_reports_sent.fetch_add(1);
  return true;
}

void usb_reenumerate() {
  tud_disconnect();
  std::this_thread::sleep_for(20ms);
  tud_connect();
}

bool usb_write_vendor(std::span<const uint8_t> frame) {
#if CONFIG_DONGLE_USB_VENDOR_INTERFACE
  if (!usb || !mounted.load())
    return false;
  // Replies come from the dispatcher worker, but the monitor service streams
  // from its own task: serialize the writers so frames never interleave.
  static std::mutex vendor_tx_mutex;
  std::lock_guard<std::mutex> lock(vendor_tx_mutex);
  std::error_code ec;
  return usb->write_vendor(frame, ec);
#else
  (void)frame;
  return false;
#endif
}

bool start_usb(const UsbHidConfig &config) {
  on_mount_callback = config.on_mount;
  on_unmount_callback = config.on_unmount;

  espp::UsbDevice::Config cfg;
  cfg.vid = config.vid;
  cfg.pid = config.pid;
  cfg.manufacturer = config.manufacturer;
  cfg.product = config.product;
  // a stable, per-device serial (derived from the base MAC) so hosts and the
  // browser console can tell dongles apart
  uint8_t mac[6] = {};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  cfg.serial_number = fmt::format("{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}", mac[0], mac[1], mac[2],
                                  mac[3], mac[4], mac[5]);
  cfg.log_level = espp::Logger::Verbosity::WARN;
  cfg.bcd_device = config.bcd_device;
  cfg.max_power_ma = config.max_power_ma;
  cfg.remote_wakeup = config.remote_wakeup;
  cfg.hid = config.hid;

#if CONFIG_DONGLE_USB_VENDOR_INTERFACE
  espp::UsbDevice::VendorFunction vendor;
  vendor.interface_name = "Dongle Console";
  vendor.webusb = true; // BOS / WebUSB / MS OS 2.0 descriptors (driverless on Windows)
  vendor.landing_page_url = "finger563.github.io/esp-usb-ble-hid/dongle_console.html";
  vendor.on_receive = on_vendor_receive;
  cfg.vendor = vendor;
#endif

#if CONFIG_DONGLE_USB_CDC_CONSOLE
  espp::UsbDevice::CdcFunction cdc;
  cdc.interface_name = "Dongle Log Console";
  cdc.route_console = true; // logs over the same cable (teed to UART0)
  cfg.cdc = cdc;
#endif

  usb = std::make_unique<espp::UsbDevice>(cfg);
#if CONFIG_DONGLE_USB_VENDOR_INTERFACE
  usb_dispatcher(); // exists before the first vendor byte can arrive
#endif

  usb->set_mount_callback([]() {
    logger.info("USB mounted");
    mounted.store(true);
    if (on_mount_callback)
      on_mount_callback();
#if CONFIG_DONGLE_USB_VENDOR_INTERFACE
    // a frame half-parsed before the (re)connect belongs to the old link
    usb_dispatcher().request_reset();
#endif
  });
  usb->set_unmount_callback([]() {
    logger.info("USB unmounted");
    // the bridge first (e.g. it timestamps the link loss), then the flag: a
    // sender that sees mounted == false must already see that state
    if (on_unmount_callback)
      on_unmount_callback();
    mounted.store(false);
#if CONFIG_DONGLE_USB_VENDOR_INTERFACE
    usb_dispatcher().request_reset();
#endif
  });

  std::error_code ec;
  if (!usb->initialize(ec)) {
    logger.error("Failed to initialize USB device: {}", ec.message());
    usb.reset();
    return false;
  }

  logger.info("USB initialization DONE ({:04x}:{:04x}, serial {})", cfg.vid, cfg.pid,
              cfg.serial_number);
  return true;
}
