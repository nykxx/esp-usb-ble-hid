# Local copy of espp/usb_device

This directory is a copy of the [espp](https://github.com/esp-cpp/espp)
`usb_device` component, version 1.3.6 (MIT license, William Emfinger), taken
from `managed_components/espp__usb_device`. It is only used by the HID
passthrough builds (see `sdkconfig.defaults.hid_passthrough`), which add this
directory with `-D DONGLE_EXTRA_COMPONENT_DIRS=components_hid_passthrough`; as a
project component it then takes precedence over the managed one. Those builds
use their own lock file (`dependencies.hid_passthrough.lock`), so the default
`dependencies.lock` is not touched.

It adds a second, optional HID function for a boot keyboard (BIOS / UEFI
support), without changing the behavior when that function is not used:

- `HidFunction::boot_protocol`: marks a HID interface as a boot device
  (bInterfaceSubClass 1, bInterfaceProtocol 1 = keyboard / 2 = mouse).
- `Config::boot_hid`: a second HID function, placed first in the configuration
  (interface 0, TinyUSB HID instance 0). Needs `CONFIG_TINYUSB_HID_COUNT=2`
  together with `Config::hid`.
- `UsbDevice::write_boot_hid_report()` and
  `UsbDevice::boot_hid_protocol_active()` (the host sent SET_PROTOCOL(BOOT)).
- The TinyUSB HID callbacks route by HID instance; `write_hid_report()` and
  `is_hid_ready()` use the HID function's instance.
- Fix: `kMaxInEndpoints` is 4, not 5. TinyUSB's DWC2 `ep_in_count` (5 on the
  ESP32-S2/S3 full-speed controller) includes EP0, so a 5th data IN endpoint
  passes the budget check but makes SET_CONFIGURATION fail (Linux: "can't set
  config #1, error -32") and the device never comes up.

To compare with the original: `diff -ru managed_components/espp__usb_device components_hid_passthrough/espp__usb_device`.
