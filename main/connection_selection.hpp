#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace ble_control {

struct Peer {
  std::array<uint8_t, 6> address{};
  uint8_t type{0};
  bool operator==(const Peer &) const = default;
};

// One preference, with a bounded exclusive search before normal automatic
// reconnection resumes. Callers serialize access; all times are monotonic us.
class Selection {
public:
  static constexpr int64_t wait_us = 10'000'000;
  std::optional<Peer> preferred;

  void restart(int64_t now) { until_ = preferred ? now + wait_us : 0; }
  void allow_any() { until_ = 0; }
  bool waiting() const { return until_ != 0; }
  bool expire(int64_t now) {
    if (!until_ || now < until_)
      return false;
    until_ = 0;
    return true;
  }
  bool accepts(const Peer &peer) const { return !waiting() || preferred == peer; }

  // Stable address order, shared by the web list and the physical button.
  // During an outstanding selection, advance from that choice so rapid taps
  // can skip an offline device; otherwise advance from the connected device.
  std::optional<Peer> next(std::vector<Peer> peers, std::optional<Peer> connected) const {
    if (peers.empty())
      return std::nullopt;
    std::sort(peers.begin(), peers.end(), less);
    const auto current = waiting() ? preferred : (connected ? connected : preferred);
    auto it = current ? std::find(peers.begin(), peers.end(), *current) : peers.end();
    if (it == peers.end() || ++it == peers.end())
      return peers.front();
    return *it;
  }
  static bool less(const Peer &a, const Peer &b) {
    if (a.address == b.address)
      return a.type < b.type;
    return std::lexicographical_compare(a.address.rbegin(), a.address.rend(), b.address.rbegin(),
                                        b.address.rend());
  }

private:
  int64_t until_{0};
};

// The edge callback and the periodic timer share this under a mutex. Debounce
// both edges; an ignored bounce must never swallow the start of a long hold.
class Button {
public:
  enum class Action { None, Next, Pair };
  static constexpr int64_t hold_us = 3'000'000;
  static constexpr int64_t debounce_us = 30'000;

  Action edge(bool active, int64_t now) {
    const auto action = tick(now);
    if (active != raw_down_) {
      raw_down_ = active;
      changed_ = now;
    }
    return action;
  }
  Action tick(int64_t now) {
    if (raw_down_ != down_ && now - changed_ >= debounce_us) {
      down_ = raw_down_;
      if (down_) {
        started_ = changed_;
        held_ = false;
      } else if (!held_) {
        // Use the release edge's time, not the possibly delayed timer tick.
        return changed_ - started_ >= hold_us ? Action::Pair : Action::Next;
      }
    }
    if (!down_ || !raw_down_ || held_ || now - started_ < hold_us)
      return Action::None;
    held_ = true;
    return Action::Pair;
  }

private:
  bool raw_down_{false};
  bool down_{false};
  bool held_{false};
  int64_t started_{0};
  int64_t changed_{0};
};

} // namespace ble_control
