#pragma once

// The bridge: what the dongle does with the BLE device's HID reports and what
// it presents on USB. Exactly one implementation is built, selected in
// menuconfig (Bridge Configuration -> Bridge mode):
//  - bridge_switch_pro.cpp: an Xbox controller presented as a Switch Pro
//    controller (the default).
//
// main.cpp owns the board, the services, BLE and USB bring-up, and routes
// their events here.

#include <cstdint>
#include <span>
#include <string>

#include "ble.hpp"
#include "usb.hpp"

/// Create the bridge state. Called once from app_main, after the services
/// (NVS) are up and before BLE / USB start.
void bridge_init();

/// What to present on USB (identity + HID function), for start_usb().
UsbHidConfig bridge_usb_config();

/// A HID report notified by the BLE device (NimBLE host task).
void bridge_on_ble_report(NimBLERemoteCharacteristic *chr, const uint8_t *data, size_t length);

/// The BLE device's battery level, in percent (NimBLE host task).
void bridge_on_ble_battery(uint8_t percent);

/// The BLE device went away: neutralize everything forwarded to the host.
void bridge_on_ble_disconnect();

/// Whether the host is consuming the forwarded reports.
bool bridge_hid_ready();

/// Called about once a second from the main loop.
void bridge_poll();

/// A short label for the connected device ("" if none): shown on the display
/// and reported to the console.
std::string bridge_device_label();
