#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include "esp_app_desc.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "logger.hpp"
#include "task.hpp"
#include "timer.hpp"

#include "ble.hpp"
#include "bridge.hpp"
#include "bsp.hpp"
#include "services.hpp"
#include "status_led.hpp"
#include "usb.hpp"

using namespace std::chrono_literals;

/************* App Configuration ****************/

#if HAS_DISPLAY
static std::shared_ptr<Gui> gui;
#endif
static std::atomic<int> battery_level_percent{100};
/********* BLE callbacks ***************/

/** Notification / Indication receiving handler callback */
void notifyCB(NimBLERemoteCharacteristic *pRemoteCharacteristic, uint8_t *pData, size_t length,
              bool isNotify) {
  // if it's the battery level characteristic, then store the battery level and
  // return.
  if (pRemoteCharacteristic->getUUID().equals(
          NimBLEUUID(espp::BatteryService::BATTERY_LEVEL_CHAR_UUID))) {
    battery_level_percent = pData[0];
    bridge_on_ble_battery(pData[0]);
    return;
  }
  // otherwise this is a HID report: the bridge forwards it to the USB host
  bridge_on_ble_report(pRemoteCharacteristic, pData, length);

  // Debugging aid (off by default): toggle the LED on every report that
  // reaches the USB side, so input activity is visible on the dongle. The
  // default is the steady "connected" level, owned by the BLE link supervisor.
  if (usb_is_mounted())
    led_blink_toggle(); // no-op unless the blink setting is on
}

/********* Dongle console (device-config module) callbacks ***************/

static device_config::Info device_info() {
  device_config::Info info;
  const auto *desc = esp_app_get_description();
  info.project = desc->project_name;
  info.firmware = desc->version;
  info.idf_version = desc->idf_ver;
  info.hardware = HARDWARE_NAME;
  info.uptime_s = static_cast<uint32_t>(esp_timer_get_time() / 1000000);
  info.usb_mounted = usb_is_mounted();
  info.ble_connected = is_ble_subscribed();
  info.ble_scanning = is_ble_scanning();
  info.pairing = is_ble_pairing();
  info.bond_count = ble_bond_count();
  // battery / serial describe the connected controller only: report them as
  // unknown (0 / "") rather than stale cached values when nothing is connected
  if (info.ble_connected) {
    info.battery_percent = static_cast<uint8_t>(battery_level_percent.load());
    info.controller = bridge_device_label();
  }
  // link diagnostics: is the controller actually sending, and is the Switch
  // actually consuming what we stream?
  info.ble_notifications = ble_notification_count();
  info.ble_last_notify_age_ms = ble_ms_since_last_notification();
  info.usb_hid_reports = usb_hid_reports_sent();
  info.usb_hid_ready = bridge_hid_ready();
  // where the controller link is (and why it is there), for the console
  using LinkState = device_config::Info::LinkState;
  switch (ble_link_state()) {
  case BleLinkState::Idle:
    info.link_state = static_cast<uint8_t>(LinkState::Idle);
    break;
  case BleLinkState::Scanning:
    info.link_state = static_cast<uint8_t>(LinkState::Scanning);
    break;
  case BleLinkState::Connecting:
    info.link_state = static_cast<uint8_t>(LinkState::Connecting);
    break;
  case BleLinkState::Encrypting:
    info.link_state = static_cast<uint8_t>(LinkState::Encrypting);
    break;
  case BleLinkState::Subscribing:
    info.link_state = static_cast<uint8_t>(LinkState::Subscribing);
    break;
  case BleLinkState::Subscribed:
    info.link_state = static_cast<uint8_t>(LinkState::Subscribed);
    break;
  }
  info.link_detail = ble_link_detail();
  return info;
}

static bool device_action(device_config::Action action, std::string &error) {
  switch (action) {
  case device_config::Action::StartPairing:
    start_ble_pairing_thread(notifyCB);
    return true;
  case device_config::Action::ClearBonds:
    if (!ble_clear_bonds()) {
      error = "could not clear the bond store";
      return false; // keep the names: their bonds may still exist
    }
    services_clear_bond_names();
    // no bonds left, so this enters pairing mode
    start_ble_reconnection_thread(notifyCB);
    return true;
  case device_config::Action::Reboot:
    // let the OK reply reach the host first
    std::thread([]() {
      std::this_thread::sleep_for(500ms);
      esp_restart();
    }).detach();
    return true;
  }
  error = "unknown action";
  return false;
}

static std::vector<device_config::BondInfo> device_bonds() {
  std::vector<device_config::BondInfo> bonds;
  for (const auto &b : ble_bonds()) {
    device_config::BondInfo info;
    info.address = b.address;
    info.address_type = b.address_type;
    info.connected = b.connected;
    // the name the controller reported when it last connected ("" = unknown)
    info.label = services_bond_name(b.address);
    bonds.push_back(std::move(info));
  }
  return bonds;
}

static bool device_forget_bond(const std::array<uint8_t, 6> &address, uint8_t address_type,
                               std::string &error) {
  if (!ble_forget_bond(address, address_type)) {
    error = "no such paired controller";
    return false;
  }
  services_forget_bond_name(address);
  if (ble_bond_count() == 0) {
    // nothing left to reconnect to: enter pairing mode
    start_ble_reconnection_thread(notifyCB);
  }
  return true;
}

static void apply_settings(const device_config::Settings &s) {
  set_led_brightness_percent(s.led_brightness);
  set_led_connected_brightness_percent(s.led_connected_brightness);
  set_led_activity_blink(s.led_activity_blink);
  // the other settings are read per input report (push_inputs); the BLE name
  // is read at boot (init_ble)
}

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "ESP USB BLE HID", .level = espp::Logger::Verbosity::DEBUG});

  logger.info("Bootup");

  // MARK: BSP initialization
  auto &bsp = Bsp::get();

  // MARK: LED initialization
  bsp.initialize_led();
  set_led(espp::Rgb(0.0f, 0.0f, 0.0f));

  // MARK: Display initialization
#if HAS_DISPLAY
  logger.info("Display initialization");
  // initialize the LCD
  if (!bsp.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be a full screen buffer
  static constexpr size_t pixel_buffer_size = bsp.lcd_width() * bsp.lcd_height();
  // initialize the LVGL display for the T-Dongle-S3
  if (!bsp.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  // initialize the gui
  logger.info("Making GUI");
  gui = std::make_shared<Gui>(Gui::Config{.log_level = espp::Logger::Verbosity::INFO});
  gui->set_label_text("");
#else  // HAS_DISPLAY
  logger.info("No display");
#endif // HAS_DISPLAY

  // MARK: Services (NVS settings, OTA, crash dumps, device config on the USB
  // vendor stream). Registered before USB starts so the modules exist as soon
  // as the host can talk to us.
  logger.info("Services initialization");
#if CONFIG_DONGLE_USB_VENDOR_INTERFACE
  auto *vendor_link = &usb_dispatcher();
#else
  espp::DispatcherWorker *vendor_link = nullptr; // plain HID build: no console stream
#endif
  services_init(vendor_link, {.info = device_info,
                              .on_action = device_action,
                              .bonds = device_bonds,
                              .forget_bond = device_forget_bond,
                              .on_settings_changed = apply_settings});
  const auto settings = services_settings();
  apply_settings(settings);
  if (const auto report = services_crash_report(); !report.empty())
    logger.warn("Previous boot crashed:\n{}", report);

  // MARK: BLE pairing timer (for use with button)
  espp::HighResolutionTimer ble_pairing_timer{
      {.name = "Pairing Timer", .callback = [&]() { start_ble_pairing_thread(notifyCB); }}};

  // MARK: Pairing button initialization
  logger.info("Initializing the button");
  auto on_button_pressed = [&](const auto &event) {
    if (event.active) {
      // start ble pairing timer
      ble_pairing_timer.oneshot(3'000'000); // 3 seconds
    } else {
      // cancel the ble pairing timer
      ble_pairing_timer.stop();
    }
  };
  bsp.initialize_button(on_button_pressed);

  // MARK: Bridge initialization (Bridge Configuration -> Bridge mode)
  bridge_init();

  // MARK: BLE initialization -- before USB: as soon as the device is mounted
  // the console can ask for status / bonds / actions, which use NimBLE.
  logger.info("BLE initialization (name '{}')", settings.ble_name);
  init_ble(settings.ble_name);
  // remember each controller's name so the console can list it by name
  ble_set_bond_name_callback(
      [](const std::array<uint8_t, 6> &address, uint8_t, const std::string &name) {
        services_set_bond_name(address, name);
      });
  // when the controller goes away (powered off, out of range, ...), the bridge
  // neutralizes its inputs: the host would otherwise keep seeing whatever was
  // last reported, forever
  ble_set_disconnect_callback([]() {
    battery_level_percent = 0;
    bridge_on_ble_disconnect();
  });

  // MARK: USB initialization
  logger.info("USB initialization");
  if (!start_usb(bridge_usb_config())) {
    logger.error("USB initialization failed");
  }
  logger.info("Scanning for peripherals");
  start_ble_reconnection_thread(notifyCB);

  // Loop here until we find a device we want to connect to
  while (true) {
    // sleep for a bit
    std::this_thread::sleep_for(1s);

    // update the display if we have one
#if HAS_DISPLAY
    // show the usb icon if the USB is mounted
    gui->set_usb_connected(usb_is_mounted());
    // show the BLE icon if the BLE subsystem is subscribed (receiving data)
    gui->set_ble_connected(is_ble_subscribed());
#endif // HAS_DISPLAY

    bridge_poll();
#if HAS_DISPLAY
    gui->set_label_text(bridge_device_label());
#endif // HAS_DISPLAY
  }
}
