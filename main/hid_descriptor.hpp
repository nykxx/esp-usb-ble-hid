#pragma once

// HID report descriptor presented on USB.
//
// The dongle mirrors the Report Map (0x2A4B) of the BLE keyboard / mouse it is
// paired with, so every BLE input report can be forwarded to the host byte for
// byte, with its report id. The USB descriptor is fixed at enumeration time, so
// the Report Map is cached in NVS: when a newly paired device's map differs
// from the cached one, it is stored and the dongle restarts once to enumerate
// with it. Until a map is cached, a plain keyboard (+ LEDs) descriptor is used.

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

/// The descriptor to present on USB: the cached Report Map, or the default.
std::vector<uint8_t> hid_descriptor_load();

/// Whether the descriptor in use came from a paired device (vs. the default).
bool hid_descriptor_is_cached();

/// Store a device's Report Map. Returns true if it differs from the descriptor
/// currently in use (the caller then restarts to enumerate with it).
bool hid_descriptor_store(std::span<const uint8_t> report_map);

/// Forget the cached Report Map (back to the default descriptor on next boot).
void hid_descriptor_clear();

/// Whether the descriptor declares report ids (Report ID items). Without them
/// reports are sent with id 0 (no id byte on the wire).
bool hid_descriptor_uses_report_ids(std::span<const uint8_t> descriptor);

/// Restarts triggered by a descriptor change since the last normal boot; used
/// to stop a restart loop if a device keeps returning a different map.
uint32_t hid_descriptor_restart_count();
void hid_descriptor_note_restart();
void hid_descriptor_reset_restart_count();

/// Where the keyboard keys are in a device's input report, found by parsing its
/// Report Map: what the boot keyboard (BIOS / UEFI) needs to rebuild a boot
/// report from it. Bit offsets count from the first byte after the report id.
struct HidKeyboardLayout {
  uint8_t report_id{0};
  int modifiers_bit{-1};        ///< 8 one-bit modifiers (LeftControl..RightGUI), -1 = none
  int keys_bit{-1};             ///< array of 8-bit key usages, -1 = none
  uint8_t keys_count{0};        ///< number of array slots
  int bitmap_bit{-1};           ///< one bit per key usage (NKRO), -1 = none
  uint16_t bitmap_usage_min{0}; ///< usage of the bitmap's first bit
  uint16_t bitmap_count{0};     ///< number of bitmap bits
};

/// Find the keyboard input report in a Report Map (nullopt if there is none).
std::optional<HidKeyboardLayout>
hid_descriptor_keyboard_layout(std::span<const uint8_t> descriptor);

/// The boot keyboard report descriptor: modifiers, reserved byte, 6 keys and
/// the LED output report, without report ids (HID 1.11, appendix B.1).
std::span<const uint8_t> hid_boot_keyboard_descriptor();

/// Rebuild the 8-byte boot keyboard report from a device keyboard report.
/// More than 6 keys pressed reports ErrorRollOver (0x01) in every slot.
std::array<uint8_t, 8> hid_boot_keyboard_report(const HidKeyboardLayout &layout,
                                                std::span<const uint8_t> report);
