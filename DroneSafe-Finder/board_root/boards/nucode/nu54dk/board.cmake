# Copyright (c) 2024 Nordic Semiconductor ASA
# Modifications Copyright (c) 2026 Quantum (NUCODE)
# SPDX-License-Identifier: Apache-2.0
#
# Nordic nRF54L15 DK 보드 설정을 NU54DK용으로 수정한 파일입니다.

if(BOARD_QUALIFIERS STREQUAL "nrf54l15/cpuapp/nu54dk")
  board_runner_args(pyocd "--target=nrf54l")

  set(nu54dk_flash_runner)
  set(nu54dk_debug_runner)

  if(SYSBUILD)
    get_property(nu54dk_sysbuild_name TARGET sysbuild_cache PROPERTY SYSBUILD_NAME)
    get_property(nu54dk_local_flash_runner TARGET sysbuild_cache
                 PROPERTY "${nu54dk_sysbuild_name}_BOARD_FLASH_RUNNER")
    get_property(nu54dk_global_flash_runner TARGET sysbuild_cache
                 PROPERTY BOARD_FLASH_RUNNER)
    get_property(nu54dk_local_debug_runner TARGET sysbuild_cache
                 PROPERTY "${nu54dk_sysbuild_name}_BOARD_DEBUG_RUNNER")
    get_property(nu54dk_global_debug_runner TARGET sysbuild_cache
                 PROPERTY BOARD_DEBUG_RUNNER)

    if(nu54dk_local_flash_runner)
      set(nu54dk_flash_runner ${nu54dk_local_flash_runner})
    elseif(nu54dk_global_flash_runner)
      set(nu54dk_flash_runner ${nu54dk_global_flash_runner})
    endif()

    if(nu54dk_local_debug_runner)
      set(nu54dk_debug_runner ${nu54dk_local_debug_runner})
    elseif(nu54dk_global_debug_runner)
      set(nu54dk_debug_runner ${nu54dk_global_debug_runner})
    endif()
  else()
    if(DEFINED CACHE{BOARD_FLASH_RUNNER})
      set(nu54dk_flash_runner $CACHE{BOARD_FLASH_RUNNER})
    endif()

    if(DEFINED CACHE{BOARD_DEBUG_RUNNER})
      set(nu54dk_debug_runner $CACHE{BOARD_DEBUG_RUNNER})
    endif()
  endif()

  if(nu54dk_flash_runner)
    board_set_flasher(${nu54dk_flash_runner})
  else()
    board_set_flasher(pyocd)
  endif()

  if(nu54dk_debug_runner)
    board_set_debugger(${nu54dk_debug_runner})
  else()
    board_set_debugger(pyocd)
  endif()

  include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
endif()
