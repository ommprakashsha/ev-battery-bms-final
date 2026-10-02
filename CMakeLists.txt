cmake_minimum_required(VERSION 3.16)
project(ev_battery_bms VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 11)

add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
    -O2
)

include_directories(
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/kernel
)

# Main BMS daemon binary
add_executable(bms_daemon src/bms_daemon_main.cpp)
target_link_libraries(bms_daemon PRIVATE pthread rt)
target_compile_definitions(bms_daemon PRIVATE
    $<$<PLATFORM_ID:Linux>:__linux__>
)

# Terminal dashboard monitor binary
add_executable(bms_dashboard src/bms_dashboard.cpp)
target_link_libraries(bms_dashboard PRIVATE pthread rt)

# Driver control and testing tool
add_executable(bms_ctl tools/bms_ctl.c)

install(TARGETS bms_daemon bms_dashboard bms_ctl
    RUNTIME DESTINATION /usr/local/bin
)
