// c++ -std=c++20 -Wall -Wextra -Werror -I main main/test/connection_selection_test.cpp -o
// /tmp/connection_selection_test
#include <cassert>
#include <cstdio>

#include "connection_selection.hpp"

using namespace ble_control;

int main() {
  const Peer a{{1, 0, 0, 0, 0, 1}, 0}, b{{2, 0, 0, 0, 0, 2}, 1}, c{{3, 0, 0, 0, 0, 3}, 0};
  const std::vector<Peer> peers{c, a, b};
  Selection s;
  assert(s.accepts(a) && s.accepts(b)); // existing auto-connect before a choice
  assert(!s.next({}, std::nullopt));
  assert(s.next(peers, std::nullopt) == a);
  assert(s.next(peers, a) == b);
  assert(s.next(peers, c) == a); // wraps in the same order as the web list
  assert(s.next({a}, a) == a);

  s.preferred = b;
  s.restart(100);
  assert(s.accepts(b) && !s.accepts(a));
  assert(!s.expire(100 + Selection::wait_us - 1));
  assert(s.next(peers, a) == c); // second tap while b is still being connected
  assert(s.expire(100 + Selection::wait_us));
  assert(s.accepts(a) && s.accepts(c)); // offline preferred device cannot block auto-connect
  assert(s.preferred == b);             // preference survives fallback
  assert(s.next(peers, c) == a);        // after fallback, cycle from the actual connection
  s.restart(20'000'000); // reboot / disconnection gives the saved preference another chance
  assert(!s.accepts(a));
  s.allow_any(); // pairing bypasses the preference
  assert(s.accepts(a));
  s.preferred.reset(); // forgetting the preferred device
  s.restart(30'000'000);
  assert(s.accepts(c) && !s.waiting());
  const Peer other_type{a.address, 1};
  s.preferred = a;
  s.restart(0);
  assert(!s.accepts(other_type)); // public and random addresses are distinct

  using Action = Button::Action;
  Button button;
  assert(button.edge(false, 0) == Action::None); // initial release
  assert(button.edge(true, 100'000) == Action::None);
  assert(button.edge(true, 110'000) == Action::None); // duplicate edge
  assert(button.edge(false, 200'000) == Action::None);
  assert(button.edge(true, 205'000) == Action::None); // release bounce
  assert(button.edge(false, 210'000) == Action::None);
  assert(button.tick(240'000) == Action::Next);
  assert(button.tick(250'000) == Action::None);
  assert(button.edge(true, 300'000) == Action::None);
  assert(button.tick(3'299'999) == Action::None);
  assert(button.tick(3'300'000) == Action::Pair);
  assert(button.tick(4'000'000) == Action::None);        // once per long hold
  assert(button.edge(false, 4'100'000) == Action::None); // never Next after Pair
  assert(button.edge(true, 5'000'000) == Action::None);
  assert(button.edge(false, 8'000'000) == Action::Pair); // release when timer was delayed
  assert(button.tick(8'100'000) == Action::None);
  assert(button.edge(true, 9'000'000) == Action::None);
  assert(button.edge(false, 11'999'999) == Action::None); // just below threshold
  assert(button.tick(12'050'000) == Action::Next); // delayed tick must not turn it into a hold
  assert(button.edge(true, 12'100'000) == Action::None);
  assert(button.edge(false, 12'100'010) == Action::None); // glitch
  assert(button.tick(12'200'000) == Action::None);
  assert(button.edge(true, 13'000'000) == Action::None);
  assert(button.edge(false, 13'001'000) == Action::None); // press bounce
  assert(button.edge(true, 13'002'000) == Action::None);
  assert(button.tick(16'002'000) == Action::Pair); // the press must not get lost
  assert(button.edge(false, 17'000'000) == Action::None);
  assert(button.tick(17'100'000) == Action::None);
  std::puts("Connection selection and button tests passed");
}
