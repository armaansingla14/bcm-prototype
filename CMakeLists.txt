cmake_minimum_required(VERSION 3.16)
project(bcm_prototype LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# --- CAN message layer -------------------------------------------------------
add_library(bcm_can
  src/can/signal.cpp
  src/can/bcm_db.cpp
  src/can/dbc_parser.cpp
  src/can/rx_monitor.cpp
  src/can/can_if.cpp)
target_include_directories(bcm_can PUBLIC include)

# --- Application (state machines) --------------------------------------------
add_library(bcm_app
  src/app/debounce.cpp
  src/app/power_mode.cpp
  src/app/turn_signal.cpp
  src/app/lighting.cpp
  src/app/door_locks.cpp
  src/app/wipers.cpp
  src/app/bcm.cpp
  src/app/naive_bcm.cpp)
target_link_libraries(bcm_app PUBLIC bcm_can)

# --- HAL backends ------------------------------------------------------------
# Host simulation backend plus vehicle/plant models used by tests and the
# scenario runner.
add_library(bcm_sim
  src/hal/sim_hal.cpp
  src/sim/vehicle_sim.cpp
  src/sim/scenario.cpp
  src/sim/requirements.cpp)
target_link_libraries(bcm_sim PUBLIC bcm_app)

# STM32 HAL adapter compiled against stub HAL headers so the target-side glue
# is type-checked on the host. The stubs do nothing at runtime.
add_library(bcm_stm32_port
  src/hal/stm32_hal.cpp
  port/stm32/stm32_hal_stub.c)
target_include_directories(bcm_stm32_port PUBLIC port/stm32)
target_link_libraries(bcm_stm32_port PUBLIC bcm_app)

# --- Tools -------------------------------------------------------------------
add_executable(scenario_runner tools/scenario_runner.cpp)
target_link_libraries(scenario_runner PRIVATE bcm_sim)

# --- Tests -------------------------------------------------------------------
enable_testing()
set(BCM_TESTS
  test_signal
  test_dbc
  test_rx_monitor
  test_can_if
  test_power_mode
  test_turn_signal
  test_lighting
  test_door_locks
  test_wipers
  test_bcm_integration
  test_fault_injection
  test_stm32_port)
foreach(t ${BCM_TESTS})
  add_executable(${t} tests/${t}.cpp)
  target_link_libraries(${t} PRIVATE bcm_sim bcm_stm32_port)
  target_include_directories(${t} PRIVATE tests)
  target_compile_definitions(${t} PRIVATE BCM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
  add_test(NAME ${t} COMMAND ${t})
endforeach()
