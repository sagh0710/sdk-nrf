/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <errno.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/hci_types.h>

#include <bluetooth/dtm_twowire/dtm_twowire_types.h>
#include <bluetooth/dtm_twowire/dtm_twowire_to_hci.h>

LOG_MODULE_REGISTER(dtm_tw_to_hci, CONFIG_DTM_TWOWIRE_TO_HCI_LOG_LEVEL);

/* Return value for internal functions that return two-wire events to signal
 * that an HCI command was generated instead. */
#define HCI_GENERATED 0xFFFF

/* Helper macros to parse DTM two-wire commands */
#define TW_CMD_CODE(tw_cmd) ((tw_cmd >> 14) & 0x03)
#define TW_CMD_CHANNEL(tw_cmd) ((tw_cmd >> 8) & 0x3F)
#define TW_CMD_LENGTH(tw_cmd) ((tw_cmd >> 2) & 0x3F)
#define TW_CMD_PKT_TYPE(tw_cmd) (tw_cmd & 0x03)
#define TW_CMD_CONTROL(tw_cmd) ((tw_cmd >> 8) & 0x3F)
#define TW_CMD_PARAMETER(tw_cmd) (tw_cmd & 0xFF)

/* DTM parameters buffered from two-wire Test Setup commands to be used in HCI commands */
struct {
  uint8_t test_data_length;           /**< Length of the payload for Transmitter Test HCI command */
  uint8_t tx_phy;                     /**< PHY for Transmitter Test HCI command */
  uint8_t rx_phy;                     /**< PHY for Receiver Test HCI command */
  uint8_t modulation_index;           /**< Modulation index for Receiver Test HCI command */
  int8_t transmit_power;              /**< Transmit power for Transmitter Test HCI command */
} dtm_hci_parameters;


static uint16_t on_test_setup_cmd(enum dtm_tw_ctrl_code control, uint8_t parameter, struct net_buf *hci_cmd)
{
  /* TODO: Implement test setup command handling, returning HCI_GENERATED if an HCI command is generated in hci_cmd,
   * and otherwise returning a two-wire event. */
  return DTM_TW_EVENT_TEST_STATUS_ERROR;
}

static uint16_t on_test_end_cmd(enum dtm_tw_ctrl_code control, uint8_t parameter, struct net_buf *hci_cmd)
{
  /* TODO: Implement test end command handling, returning HCI_GENERATED if an HCI command is generated in hci_cmd,
   * and otherwise returning a two-wire error event. */
  return DTM_TW_EVENT_TEST_STATUS_ERROR;
}

static uint16_t on_test_rx_cmd(uint8_t channel, struct net_buf *hci_cmd)
{
  /* TODO: Implement test receive command handling, returning HCI_GENERATED if an HCI command is generated in hci_cmd,
   * and otherwise returning a two-wire error event. */
  return DTM_TW_EVENT_TEST_STATUS_ERROR;
}

static uint16_t on_test_tx_cmd(uint8_t channel, uint8_t length, enum dtm_tw_pkt_type type,
                               struct net_buf *hci_cmd)
{
  /* TODO: Implement test transmit command handling, returning HCI_GENERATED if an HCI command is generated in hci_cmd,
   * and otherwise returning a two-wire error event. */
  return DTM_TW_EVENT_TEST_STATUS_ERROR;
}

static int dtm_tw_to_hci_init(void)
{
  /* TODO: set the buffered DTM parameters to default values */

  return 0;
}

dtm_tw_to_hci_status_t dtm_tw_to_hci_process_tw_cmd(const uint16_t tw_cmd, struct net_buf *hci_cmd, uint16_t *tw_event)
{
  if (!tw_event || !hci_cmd) {
    return DTM_TW_TO_HCI_STATUS_ERROR;
  }

  enum dtm_tw_cmd_code cmd_code = TW_CMD_CODE(tw_cmd);

  /* RX and TX test commands */
  uint8_t channel = TW_CMD_CHANNEL(tw_cmd);
  uint8_t length = TW_CMD_LENGTH(tw_cmd);
  enum dtm_tw_pkt_type pkt_type = TW_CMD_PKT_TYPE(tw_cmd);

  /* Setup and End commands */
  enum dtm_tw_ctrl_code control = TW_CMD_CONTROL(tw_cmd);
  uint8_t parameter = TW_CMD_PARAMETER(tw_cmd);

  switch (cmd_code) {
  case DTM_TW_CMD_TEST_SETUP:
    LOG_DBG("Executing test setup command. Control: %d Parameter: %d", control,
                      parameter);
    *tw_event = on_test_setup_cmd(control, parameter, hci_cmd);
    break;

  case DTM_TW_CMD_TEST_END:
    LOG_DBG("Executing test end command. Control: %d Parameter: %d", control,
                      parameter);
    *tw_event = on_test_end_cmd(control, parameter, hci_cmd);
    break;

  case DTM_TW_CMD_RECEIVER_TEST:
    LOG_DBG("Executing reception test command. Channel: %d", channel);
    *tw_event = on_test_rx_cmd(channel, hci_cmd);
    break;

  case DTM_TW_CMD_TRANSMITTER_TEST:
    LOG_DBG("Executing transmission test command. Channel: %d Length: %d Type: %d", channel, length, pkt_type);
    *tw_event = on_test_tx_cmd(channel, length, pkt_type, hci_cmd);
    break;

  default:
    LOG_ERR("Received unknown command code %d", cmd_code);
    *tw_event = DTM_TW_EVENT_TEST_STATUS_ERROR;
  }

  if (*tw_event == HCI_GENERATED) {
    return DTM_TW_TO_HCI_STATUS_HCI_CMD;
  } else {
    return DTM_TW_TO_HCI_STATUS_TW_EVENT;
  }
}

dtm_tw_to_hci_status_t dtm_tw_to_hci_process_hci_event(const uint16_t tw_cmd, const struct net_buf *hci_event,
                                                       uint16_t *tw_event)
{
  if (!hci_event || !hci_event->data || !tw_event) {
    return DTM_TW_TO_HCI_STATUS_ERROR;
  }

  /* Verify that the received packet is an HCI event */
  uint8_t packet_indicator = hci_event->data[0];
  if (packet_indicator != BT_HCI_H4_EVT) {
    LOG_WRN("Received non-event HCI packet 0x%02X", packet_indicator);
    return DTM_TW_TO_HCI_STATUS_UNHANDLED;
  }

  const struct bt_hci_evt_hdr *event_hdr = (void *)(&hci_event->data[1]);

  /* Check for command status event, which is the event returned when an HCI command fails to be processed */
  if (event_hdr->evt == BT_HCI_EVT_CMD_STATUS) {
    const struct bt_hci_evt_cmd_status *status_event = (void *)event_hdr->data;
    LOG_ERR("HCI command 0x%04X failed with status 0x%02X", status_event->opcode, status_event->status);
    *tw_event = DTM_TW_EVENT_TEST_STATUS_ERROR;
    return DTM_TW_TO_HCI_STATUS_TW_EVENT;
  }

  /* Verify that the received event is a command complete event (given that it was not a status event) */
  if (event_hdr->evt != BT_HCI_EVT_CMD_COMPLETE) {
    LOG_WRN("Unhandled HCI event 0x%02X", event_hdr->evt);
    return DTM_TW_TO_HCI_STATUS_UNHANDLED;
  }

  const struct bt_hci_evt_cmd_complete *cc_event = (void *)event_hdr->data;

  /* The completed command's return parameters are at the end of the CC event */
  const void *cc_event_return_params = &cc_event[1];

  switch (cc_event->opcode) {
  /* TODO: Handle specific DTM-related HCI command complete events, put the resulting two-wire event in tw_event,
   * and return DTM_TW_TO_HCI_STATUS_TW_EVENT. */
  default:
    LOG_WRN("Unhandled HCI command complete event for opcode 0x%04X", cc_event->opcode);
    return DTM_TW_TO_HCI_STATUS_UNHANDLED;
  }
}

SYS_INIT(dtm_tw_to_hci_init, APPLICATION, 0);
