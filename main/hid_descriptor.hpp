#pragma once

// HID report descriptor presented on USB.
//
// The dongle mirrors the Report Map (0x2A4B) of the BLE keyboard / mouse it is
// paired with, so every BLE input report can be forwarded to the host byte for
// byte, with its report id. The USB descriptor is fixed at enumeration time, so
// the Report Map is cached in NVS: when a newly paired device's map differs
// from the cached one, it is stored and the dongle restarts once to enumerate
// with it. Until a map is cached, a plain keyboard (+ LEDs) descriptor is used.

#include <cstdint>
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
