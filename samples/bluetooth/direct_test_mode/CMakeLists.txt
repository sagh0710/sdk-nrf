#
# Copyright (c) 2026 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dtm)

target_sources_ifdef(CONFIG_DTM_SAMPLE_INTERFACE_TWOWIRE app PRIVATE
  src/dtm_twowire_transport.c
  src/main.c
)

target_sources_ifdef(CONFIG_DTM_SAMPLE_INTERFACE_HCI app PRIVATE
    #$ENV{ZEPHYR_BASE}/samples/bluetooth/hci_uart_async/src/hci_uart_async.c
    $ENV{ZEPHYR_BASE}/samples/bluetooth/hci_uart/src/main.c
)
