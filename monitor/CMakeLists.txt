cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(px4_safety_monitor LANGUAGES C)

set(MAVLINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/c_library_v2)

target_include_directories(app PRIVATE
  include
  ${MAVLINK_DIR}
  ${LIBMETAL_INCLUDE_DIR}
  ${OPENAMP_INCLUDE_DIR}
  ${PLATFORM_DIR}
)
target_compile_options(app PRIVATE -Wno-address-of-packed-member)

target_sources(app PRIVATE
  src/main_zephyr.c
  src/transport_rpmsg.c
  src/monitor.c
  src/params.c
  src/mavlink_rx.c
  src/state.c
  src/arbiter.c
  src/override_tx.c
  src/rules/geofence.c
  src/rules/envelope.c
  src/rules/battery.c
  src/rules/liveness.c
)
