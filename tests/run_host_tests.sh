#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

test_build_dir="${DONGLE_TEST_BUILD_DIR:-/tmp/esp-usb-ble-hid-host-tests}"
mkdir -p "$test_build_dir"
cxx="${CXX:-c++}"
flags=(-std=c++20 -Wall -Wextra -Werror)

"$cxx" "${flags[@]}" -I main main/test/connection_selection_test.cpp \
  -o "$test_build_dir/connection_selection"
"$test_build_dir/connection_selection"
"$cxx" "${flags[@]}" -I components/device_config/include \
  components/device_config/test/device_config_host_test.cpp -o "$test_build_dir/protocol"
"$test_build_dir/protocol"

# Run after idf.py reconfigure/build has fetched the pinned dependencies.
includes=(-I components/device_config/include)
for component in base_component logger format dispatcher stream_frame; do
  includes+=(-I "managed_components/espp__${component}/include")
done
includes+=(-I managed_components/espp__format/detail/fmt/include)
"$cxx" "${flags[@]}" "${includes[@]}" \
  components/device_config/test/device_selection_module_test.cpp \
  managed_components/espp__logger/src/logger.cpp -o "$test_build_dir/selection_module"
"$test_build_dir/selection_module"

"$cxx" "${flags[@]}" "${includes[@]}" -I main/test/hid_descriptor_stubs -I main \
  main/test/hid_descriptor_test.cpp main/hid_descriptor.cpp \
  managed_components/espp__logger/src/logger.cpp -o "$test_build_dir/hid_descriptor"
"$test_build_dir/hid_descriptor"

"${DONGLE_NODE:-node}" web/test/bond_selection_test.cjs
