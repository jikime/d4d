# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
set(CONF_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../common/base.conf ${CMAKE_CURRENT_SOURCE_DIR}/prj.conf)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nu54_secure_drone)
target_include_directories(app PRIVATE ../common ../common/vendor/sh2)
target_sources(app PRIVATE src/main.c ../common/secure_pair.c ../common/usb_console.c
  ../common/nu54_peripherals.c ../common/nu54_imu.c)
file(GLOB sh2_sources ../common/vendor/sh2/*.c)
target_sources(app PRIVATE ${sh2_sources})
