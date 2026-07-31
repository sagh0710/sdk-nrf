#
# Copyright (c) 2026 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

if(CONFIG_DTM_SAMPLE_INTERFACE_HCI)

  # Set the path to the board-specific overlay file, prioritizing files in hci_overlays over the ones in the hci_uart sample
  set(DTM_SAMPLE_OVERLAY_FILE_PATH ${CMAKE_CURRENT_LIST_DIR}/hci_overlays/${NORMALIZED_BOARD_TARGET}.overlay)
  set(HCI_UART_SAMPLE_OVERLAY_FILE_PATH $ENV{ZEPHYR_BASE}/samples/bluetooth/hci_uart/boards/${NORMALIZED_BOARD_TARGET}.overlay)
  if (EXISTS ${DTM_SAMPLE_OVERLAY_FILE_PATH})
    set(board_overlay ${DTM_SAMPLE_OVERLAY_FILE_PATH})
  elseif(EXISTS ${HCI_UART_SAMPLE_OVERLAY_FILE_PATH})
    set(board_overlay ${HCI_UART_SAMPLE_OVERLAY_FILE_PATH})
  else()
    message(FATAL_ERROR "Missing overlay. Expected in ${DTM_SAMPLE_OVERLAY_FILE_PATH} or ${HCI_UART_SAMPLE_OVERLAY_FILE_PATH}")
  endif()

  # Set the path to the sample config from the hci_uart sample
  # This is in addition to the DTM sample prj.conf
  set(sample_config $ENV{ZEPHYR_BASE}/samples/bluetooth/hci_uart/prj.conf)

  # Set the path to the board-specific overlay Kconfig file if it exists, prioritizing files in hci_overlays over the ones in the hci_uart sample
  set(DTM_SAMPLE_BOARD_CONFIG_FILE_PATH ${CMAKE_CURRENT_LIST_DIR}/hci_overlays/${NORMALIZED_BOARD_TARGET}.conf)
  set(HCI_UART_SAMPLE_BOARD_CONFIG_FILE_PATH $ENV{ZEPHYR_BASE}/samples/bluetooth/hci_uart/boards/${NORMALIZED_BOARD_TARGET}.conf)
  if (EXISTS ${DTM_SAMPLE_BOARD_CONFIG_FILE_PATH})
    set(board_config ${DTM_SAMPLE_BOARD_CONFIG_FILE_PATH})
  elseif(EXISTS ${HCI_UART_SAMPLE_BOARD_CONFIG_FILE_PATH})
    set(board_config ${HCI_UART_SAMPLE_BOARD_CONFIG_FILE_PATH})
  endif()

  set(${DEFAULT_IMAGE}_DTC_OVERLAY_FILE "${${DEFAULT_IMAGE}_DTC_OVERLAY_FILE};${sample_board_overlay}" CACHE INTERNAL "sysbuild controlled")
  set(${DEFAULT_IMAGE}_EXTRA_CONF_FILE "${${DEFAULT_IMAGE}_EXTRA_CONF_FILE};${sample_board_fragment}" CACHE INTERNAL "sysbuild controlled")

endif(CONFIG_DTM_SAMPLE_INTERFACE_HCI)
