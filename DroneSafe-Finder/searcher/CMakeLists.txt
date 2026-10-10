# Copyright (c) 2024 Nordic Semiconductor ASA
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
cmake_minimum_required(VERSION 3.20.0)
list(APPEND CONF_FILE
  ${CMAKE_CURRENT_SOURCE_DIR}/../common/base.conf
  ${CMAKE_CURRENT_SOURCE_DIR}/prj.conf)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nu54_secure_recovery_searcher)
target_include_directories(app PRIVATE ../common src tests)
target_sources(app PRIVATE
  src/main.c
  src/walk_range_filter.c
  tests/walk_range_filter_test.c
  ../common/secure_pair.c
  ../common/usb_console.c
  ../common/nu54_peripherals.c)
