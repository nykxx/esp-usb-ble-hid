// Host integration test using the real dispatcher framing and DeviceConfig.
// Requires managed_components from an ESP-IDF configure/build.
#include <cassert>
#include <cstdio>

#include "device_config.hpp"

namespace dc = device_config;
namespace sf = espp::stream_frame;

int main() {
  std::vector<sf::Frame> replies;
  sf::StreamParser parser;
  dc::BondInfo bond;
  bond.address = {6, 5, 4, 3, 2, 1};
  bond.address_type = 1;
  bool allow = true;
  int calls = 0;
  DeviceConfig config({
      .send =
          [&](std::span<const uint8_t> bytes) {
            auto frames = parser.feed(bytes);
            replies.insert(replies.end(), frames.begin(), frames.end());
          },
      .bonds = [&] { return std::vector{bond}; },
      .select_bond =
          [&](const auto &address, uint8_t type, std::string &error) {
            ++calls;
            assert(address == bond.address && type == bond.address_type);
            if (!allow) {
              error = "could not save selection";
              return false;
            }
            bond.selected = true;
            return true;
          },
  });
  sf::Frame request{.module = dc::kModule,
                    .type = static_cast<uint8_t>(dc::Msg::SelectBond),
                    .payload = dc::make_forget_bond_payload(bond.address, bond.address_type)};
  config.handle(request);
  assert(calls == 1 && replies.size() == 2);
  assert(replies[0].type == static_cast<uint8_t>(dc::Msg::Ok));
  assert(replies[0].payload == dc::make_ok_payload(dc::Msg::SelectBond));
  assert(replies[1].type == static_cast<uint8_t>(dc::Msg::Bonds));
  const auto bonds = dc::parse_bonds(replies[1].payload);
  assert(bonds && bonds->front().selected && !bonds->front().connected);
  // Persistence / selection failure must not claim success or publish new state.
  allow = false;
  replies.clear();
  config.handle(request);
  assert(calls == 2 && replies.size() == 1);
  assert(replies[0].type == static_cast<uint8_t>(dc::Msg::Error));
  // Invalid address type and short payload must not invoke the BLE callback.
  replies.clear();
  request.payload.back() = 2;
  config.handle(request);
  request.payload.pop_back();
  config.handle(request);
  assert(calls == 2 && replies.size() == 2);
  for (const auto &reply : replies)
    assert(reply.type == static_cast<uint8_t>(dc::Msg::Error));
  std::puts("Device selection module integration tests passed");
}
