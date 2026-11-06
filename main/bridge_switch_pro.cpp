// Bridge mode: Xbox controller (BLE) -> Nintendo Switch Pro controller (USB).
#include "sdkconfig.h"
#if CONFIG_BRIDGE_MODE_SWITCH_PRO

#include "bridge.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include "esp_timer.h"

#include "logger.hpp"
#include "task.hpp"
#include "timer.hpp"

#include "switch_pro.hpp"
#include "xbox.hpp"

#include "services.hpp"
#include "status_led.hpp"

// set to 1 to enable twirling the joysticks automatically (for testing) when
// there is no BLE device connected.
#define DEBUG_NO_BLE_TWIRL_JOYSTICKS 0
// set to 1 to enable pushing the buttons automatically (for testing) when there
// is no BLE device connected. Not recommended unless you want to annoy
// yourself.
#define DEBUG_NO_BLE_TEST_BUTTONS 0

using namespace std::chrono_literals;

static espp::Logger logger({.tag = "Switch Pro", .level = espp::Logger::Verbosity::INFO});

static std::shared_ptr<GamepadDevice> ble_gamepad;
static std::shared_ptr<espp::SwitchPro> usb_controller;
static std::atomic<int> battery_level_percent{100};
static std::mutex serial_number_mutex;
static std::string serial_number = "";

static std::string get_serial_number() {
  std::lock_guard<std::mutex> lk(serial_number_mutex);
  return serial_number;
}
static void set_serial_number(const std::string &sn) {
  std::lock_guard<std::mutex> lk(serial_number_mutex);
  serial_number = sn;
}

/********* Input mapping ***************/

// Radial deadzone: inside it the stick reads (0,0); outside it the remaining
// travel is rescaled so full deflection still reaches 1.0.
static void apply_deadzone(GamepadInputs::Joystick &j, float deadzone) {
  if (deadzone <= 0.0f)
    return;
  const float mag = std::sqrt(j.x * j.x + j.y * j.y);
  if (mag < deadzone) {
    j.x = j.y = 0.0f;
    return;
  }
  const float scale = std::min(1.0f, (mag - deadzone) / (1.0f - deadzone)) / mag;
  j.x *= scale;
  j.y *= scale;
}

// The last inputs pushed (before settings), so the report can be re-pushed by
// the USB registration timer below; and the end of the registration window
// (0 = none) during which L+R are held. One mutex serializes every report
// push with the window transitions, so a notification arriving between the
// timer's snapshot and its re-push can never be overwritten by the snapshot.
static std::mutex inputs_mutex;
static GamepadInputs last_inputs{};
static int64_t registration_until_us{0}; // guarded by inputs_mutex
static constexpr int64_t kRegistrationWindowUs = 1000 * 1000;

// Apply the user settings to the controller inputs and push them into the USB
// controller's input report (streamed to the host by the USB sender task).
static void push_inputs_locked(GamepadInputs inputs) {
  // The Switch's Change Grip/Order screen registers a controller when L+R are
  // pressed: hold them for a moment after every USB enumeration (see
  // usb_registration_tick) so the dongle registers itself without a hand on
  // the wireless controller.
  if (esp_timer_get_time() < registration_until_us) {
    inputs.buttons.l1 = 1;
    inputs.buttons.r1 = 1;
  }
  const auto s = services_settings();
  if (s.invert_left_y)
    inputs.left_joystick.y = -inputs.left_joystick.y;
  if (s.invert_right_y)
    inputs.right_joystick.y = -inputs.right_joystick.y;
  const float deadzone = s.deadzone_percent / 100.0f;
  apply_deadzone(inputs.left_joystick, deadzone);
  apply_deadzone(inputs.right_joystick, deadzone);
  if (s.swap_ab) {
    const bool a = inputs.buttons.a, b = inputs.buttons.b;
    inputs.buttons.a = b;
    inputs.buttons.b = a;
  }
  if (s.swap_xy) {
    const bool x = inputs.buttons.x, y = inputs.buttons.y;
    inputs.buttons.x = y;
    inputs.buttons.y = x;
  }
  usb_controller->update_input_report([&](espp::SwitchPro::InputReport &r) {
    r.reset();
    r.set_buttons(inputs.buttons);
    r.set_left_joystick(inputs.left_joystick.x, inputs.left_joystick.y);
    r.set_right_joystick(inputs.right_joystick.x, inputs.right_joystick.y);
    r.set_brake(inputs.l2.value);
    r.set_accelerator(inputs.r2.value);
  });
  usb_controller->set_battery_level(static_cast<uint8_t>(battery_level_percent.load()));
}

static void push_inputs(GamepadInputs inputs) {
  std::lock_guard<std::mutex> lk(inputs_mutex);
  last_inputs = inputs;
  push_inputs_locked(inputs);
}

// Polled every 50 ms: when the host has just finished the handshake and
// started taking reports, hold L+R for kRegistrationWindowUs, then release.
static void usb_registration_tick() {
  static bool was_ready = false;
  const bool ready = bridge_hid_ready();
  const int64_t now = esp_timer_get_time();
  // the window transition and the re-push happen under the same lock as every
  // other push, so the latest inputs are what gets (re)pushed
  std::lock_guard<std::mutex> lk(inputs_mutex);
  if (ready && !was_ready) {
    registration_until_us = now + kRegistrationWindowUs;
    push_inputs_locked(last_inputs); // with L+R
  } else if (!ready) {
    registration_until_us = 0;
  } else if (registration_until_us && now >= registration_until_us) {
    registration_until_us = 0;
    push_inputs_locked(last_inputs); // release L+R
  }
  was_ready = ready;
}

/********* USB: handshake replies + the streamed input report ***************/

// The Switch sometimes drops the device without a new bus transaction (e.g.
// after the Change Grip/Order screen): the pull-up is re-armed after the
// link has been down this long, so the host enumerates us again.
static constexpr int64_t kReenumerateAfterUs = 2500 * 1000;
static std::atomic<bool> ever_mounted{false};
static std::atomic<int64_t> unmounted_at_us{0};

// Replies are produced on the TinyUSB task (when a host OUTPUT report arrives)
// and on mount; they are queued and sent from one sender task so
// write_hid_report() is never called from the TinyUSB task (where waiting for
// the endpoint to drain would deadlock).
struct OutReport {
  uint8_t id{0};
  std::vector<uint8_t> data;
};
static std::deque<OutReport> hid_tx_queue;
static std::mutex hid_tx_mutex;
static std::condition_variable hid_tx_cv;
static std::unique_ptr<espp::Task> hid_sender_task;

static void enqueue_hid(espp::SwitchPro::ReportData rd) {
  {
    std::lock_guard<std::mutex> lock(hid_tx_mutex);
    // cap the queue so a misbehaving host cannot grow it without bound
    if (hid_tx_queue.size() < 16)
      hid_tx_queue.push_back({rd.first, std::move(rd.second)});
  }
  hid_tx_cv.notify_one();
}

static bool hid_sender_fn(std::mutex &, std::condition_variable &) {
  OutReport rep;
  bool have = false;
  {
    std::unique_lock<std::mutex> lock(hid_tx_mutex);
    // the real Pro Controller reports every 8 ms (full-speed HID poll rate)
    hid_tx_cv.wait_for(lock, 8ms, [] { return !hid_tx_queue.empty(); });
    if (!hid_tx_queue.empty()) {
      rep = std::move(hid_tx_queue.front());
      hid_tx_queue.pop_front();
      have = true;
    }
  }
  if (!usb_is_mounted()) {
    // Re-arm the pull-up if a host that had us configured let the link drop
    // and has not re-enumerated on its own (never before the first mount, so
    // a slow initial enumeration is not interrupted). Runs on our own task.
    if (ever_mounted.load() &&
        esp_timer_get_time() - unmounted_at_us.load() >= kReenumerateAfterUs) {
      logger.info("USB link down for {} ms without re-enumeration; re-arming the pull-up",
                  kReenumerateAfterUs / 1000);
      unmounted_at_us.store(esp_timer_get_time());
      usb_reenumerate();
    }
    return false;
  }
  std::error_code ec;
  if (have) {
    // a handshake reply: this is our own task, so retry until the endpoint frees
    for (int i = 0; i < 20 && !usb_write_hid_report(rep.id, rep.data, ec); ++i)
      std::this_thread::sleep_for(1ms);
  } else if (usb_controller->is_ready()) {
    const auto report = usb_controller->get_input_report();
    if (!report.empty())
      usb_write_hid_report(usb_controller->input_report_id(), report, ec); // best-effort
  }
  return false; // don't stop the task
}

/********* bridge.hpp ***************/

void bridge_init() {
  usb_controller = std::make_shared<espp::SwitchPro>(
      espp::SwitchPro::Config{.log_level = espp::Logger::Verbosity::WARN});
  ble_gamepad = std::make_shared<Xbox>();

  hid_sender_task = espp::Task::make_unique(
      {.callback = hid_sender_fn,
       .task_config = {.name = "usb_hid_tx", .stack_size_bytes = 4096, .priority = 10}});
  hid_sender_task->start();

  // hold L+R for a moment after every USB enumeration so the Switch's Change
  // Grip/Order screen registers the dongle (see usb_registration_tick)
  static espp::Timer usb_registration_timer({.name = "usb-reg",
                                             .period = 50ms,
                                             .callback =
                                                 [] {
                                                   usb_registration_tick();
                                                   return false;
                                                 },
                                             .log_level = espp::Logger::Verbosity::WARN});
}

UsbHidConfig bridge_usb_config() {
  // The USB identity must be Nintendo's Pro Controller for a Switch to bind it.
  UsbHidConfig cfg;
  cfg.vid = espp::SwitchPro::vid;
  cfg.pid = espp::SwitchPro::pid;
  cfg.manufacturer = espp::SwitchPro::manufacturer_name;
  cfg.product = espp::SwitchPro::product_name;
  // Descriptor details a Switch compares against a real Pro Controller: device
  // release, bus power and remote wakeup as the original reports them (one
  // source of truth: espp::SwitchPro, espp >= 1.3.6).
  cfg.bcd_device = espp::SwitchPro::bcd_device;
  cfg.max_power_ma = espp::SwitchPro::max_power_ma;
  cfg.remote_wakeup = espp::SwitchPro::remote_wakeup;

  cfg.hid.interface_name = "Switch Pro Controller";
  cfg.hid.report_descriptor = usb_controller->get_report_descriptor();
  cfg.hid.has_out_endpoint = true; // receive the host's OUTPUT reports (the handshake)
  cfg.hid.poll_interval_ms = 8;    // the real Pro Controller polls at 8 ms
  cfg.hid.on_receive = [](std::span<const uint8_t> data) {
    // TinyUSB task context: compute the reply and queue it (don't send here)
    if (data.empty())
      return;
    if (auto reply = usb_controller->on_hid_report(data[0], data.data(), data.size()))
      enqueue_hid(std::move(*reply));
  };
  cfg.on_mount = []() {
    ever_mounted.store(true);
    // kick off the handshake: the controller proactively sends its device-init
    // (0x81) report
    if (auto init = usb_controller->on_attach())
      enqueue_hid(std::move(*init));
  };
  cfg.on_unmount = []() {
    // timestamp first: the sender reads it right after it sees mounted == false
    unmounted_at_us.store(esp_timer_get_time());
    std::lock_guard<std::mutex> lock(hid_tx_mutex);
    hid_tx_queue.clear();
  };
  return cfg;
}

void bridge_on_ble_report(NimBLERemoteCharacteristic *chr, const uint8_t *data, size_t length) {
  // which report this is is told by the characteristic's Report Reference (a
  // controller may notify several input reports); an unknown characteristic
  // is assumed to be the main gamepad input report
  const uint8_t report_id = ble_report_id_for(chr).value_or(ble_gamepad->get_input_report_id());
  ble_gamepad->set_report_data(report_id, data, length);
  if (report_id != ble_gamepad->get_input_report_id())
    return; // not the gamepad input report: nothing to forward

  // convert it to GamepadInputs and push it to the USB controller
  push_inputs(ble_gamepad->get_gamepad_inputs());
}

void bridge_on_ble_battery(uint8_t percent) { battery_level_percent = percent; }

void bridge_on_ble_disconnect() {
  // release every button and center the sticks: the Switch would otherwise
  // keep seeing whatever was last reported, forever
  battery_level_percent = 0;
  push_inputs(GamepadInputs{});
}

bool bridge_hid_ready() { return usb_is_mounted() && usb_controller && usb_controller->is_ready(); }

void bridge_poll() {
  if (is_ble_subscribed()) {
    // if we haven't gotten the serial number, then get that and save it
    if (get_serial_number().empty())
      set_serial_number(get_connected_client_serial_number());
    return;
  }
  // make sure to reset the connected device serial number
  set_serial_number("");

#if DEBUG_NO_BLE_TWIRL_JOYSTICKS
  // otherwise, just twirl the joysticks
  static constexpr int num_segments = 16;
  static int index = 0;
  float angle = 2.0f * M_PI * (index % num_segments) / (num_segments);

  GamepadInputs inputs{};
  // joystick inputs are in the range [-1, 1] float
  inputs.left_joystick.x = sin(angle);
  inputs.left_joystick.y = cos(angle);
  inputs.right_joystick.x = cos(angle);
  inputs.right_joystick.y = sin(angle);

#if DEBUG_NO_BLE_TEST_BUTTONS
  // NOTE: this is not recommended since it's annoying when it works, but left
  // in for debugging when it doesn't work.
  static constexpr int num_buttons = 15;
  inputs.set_button(index % num_buttons, true);
#endif // DEBUG_NO_BLE_TEST_BUTTONS

  index++;

  push_inputs(inputs);
  if (!usb_is_mounted()) {
    set_led(espp::Rgb(1.0f, 0.0f, 0.0f));
  }
#endif // DEBUG_NO_BLE_TWIRL_JOYSTICKS
}

std::string bridge_device_label() { return get_serial_number(); }

#if HAS_DISPLAY
void bridge_update_display(Gui &) {
  // the default status screen (Xbox controller -> Switch) is this mode's
}
#endif

#endif // CONFIG_BRIDGE_MODE_SWITCH_PRO
