#pragma once

// USB side of the bridge, built on espp::UsbDevice:
//  - a HID interface, described by the bridge (see bridge.hpp): the Switch Pro
//    controller, or the BLE device's own HID descriptor,
//  - optionally a vendor (WebUSB) interface carrying the espp dispatcher stream
//    (device configuration, OTA, crash dumps -- see services.hpp),
//  - optionally a CDC-ACM interface carrying the log console.

#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <system_error>

#include "sdkconfig.h"

#include "dispatcher_worker.hpp"
#include "usb_device.hpp"

#if CONFIG_DONGLE_USB_VENDOR_INTERFACE
/// The vendor stream's dispatcher + worker (espp::DispatcherWorker): register
/// services on it (at any time; it is created on first use), and every module
/// handler -- and its replies -- runs on the worker task, never on TinyUSB's.
/// Only built with the vendor interface (a plain HID build has no such stream
/// and spends no task on it).
espp::DispatcherWorker &usb_dispatcher();

/// Called on the dispatcher worker when vendor bytes had to be dropped (the
/// parser has already been reset); services use it to abort an in-flight
/// transfer.
/// May be set from any task at any time (the worker takes a copy under a lock).
void usb_set_rx_overflow_callback(std::function<void()> callback);
#endif // CONFIG_DONGLE_USB_VENDOR_INTERFACE

/// What the bridge presents on USB: the device identity and the HID function.
/// The serial number (from the MAC), the vendor and the CDC functions are added
/// by start_usb().
struct UsbHidConfig {
  uint16_t vid{0};
  uint16_t pid{0};
  std::string manufacturer;
  std::string product;
  uint16_t bcd_device{0x0100};
  uint16_t max_power_ma{100};
  bool remote_wakeup{true};
  /// The HID function; its on_receive runs on the TinyUSB task (must not block).
  espp::UsbDevice::HidFunction hid;
  /// Optional boot keyboard (BIOS / UEFI), presented as interface 0.
  std::optional<espp::UsbDevice::HidFunction> boot_hid;
  /// Called (TinyUSB task) when the host configures / drops the device.
  std::function<void()> on_mount{nullptr};
  std::function<void()> on_unmount{nullptr};
};

/// Bring up the USB device.
bool start_usb(const UsbHidConfig &config);

/// Whether the USB host has configured (mounted) the device.
bool usb_is_mounted();

/// Send one HID input report. Returns false with
/// std::errc::resource_unavailable_try_again while the previous report is still
/// in flight. Do not call from the TinyUSB task when retrying.
bool usb_write_hid_report(uint8_t report_id, std::span<const uint8_t> report, std::error_code &ec);

/// Send one boot keyboard report (8 bytes, no report id); errors as above.
bool usb_write_boot_hid_report(std::span<const uint8_t> report, std::error_code &ec);

/// Whether the host drives the boot keyboard in boot protocol (BIOS / UEFI).
bool usb_boot_protocol_active();

/// Input reports accepted by the USB stack since boot (diagnostics).
uint32_t usb_hid_reports_sent();

/// Drop and re-attach the D+ pull-up so the host enumerates the device again.
void usb_reenumerate();

/// Write a complete frame to the vendor interface (no-op if it is disabled).
/// Safe to call from any task except the TinyUSB task.
bool usb_write_vendor(std::span<const uint8_t> frame);
