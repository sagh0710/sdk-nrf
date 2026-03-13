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

/* Helper macros to check if a DTM feature is marked as supported in the HCI_LE_Read_All_Local_Supported_Features
* command response */
#define FEATURE_DLE(features) \
  BT_FEAT_LE_DLE(features)
#define FEATURE_2M_PHY(features) \
  BT_FEAT_LE_PHY_2M(features)
#define FEATURE_TX_STABLE_MOD(features) \
  BT_LE_FEAT_TEST(features, BT_LE_FEAT_BIT_SMI_TX)
#define FEATURE_CODED_PHY(features) \
  BT_FEAT_LE_PHY_CODED(features)
/* TODO: DRGN-27910 add CTE and antenna switching support */
#define FEATURE_CTE(features) 0
#define FEATURE_ANT_SWITCHING(features) 0
#define FEATURE_AOD_1US_TX(features) 0
#define FEATURE_AOD_1US_RX(features) 0
#define FEATURE_AOA_1US_RX(features) 0

/* DTM parameters buffered from two-wire Test Setup commands to be used in HCI commands */
struct {
  uint8_t test_data_length;           /**< Length of the payload for Transmitter Test HCI command */
  uint8_t tx_phy;                     /**< PHY for Transmitter Test HCI command */
  uint8_t rx_phy;                     /**< PHY for Receiver Test HCI command */
  uint8_t modulation_index;           /**< Modulation index for Receiver Test HCI command */
  int8_t transmit_power;              /**< Transmit power for Transmitter Test HCI command */
} dtm_hci_parameters;

static uint16_t reset_dtm(uint8_t parameter)
{
  if (parameter < DTM_TW_RESET_MIN_RANGE || parameter > DTM_TW_RESET_MAX_RANGE) {
    return DTM_TW_EVENT_TEST_STATUS_ERROR;
  }

  /* Reset DTM parameters to default values as specified in Core v6.2, Vol 6, Part F, 3.3.2 */
  memset(&dtm_hci_parameters, 0, sizeof(dtm_hci_parameters));
  dtm_hci_parameters.tx_phy = BT_HCI_LE_TX_PHY_1M;
  dtm_hci_parameters.modulation_index = BT_HCI_LE_MOD_INDEX_STANDARD;
  return DTM_TW_EVENT_TEST_STATUS_SUCCESS;
}

static uint16_t set_upper_length_bits(uint8_t parameter)
{
  if (parameter < DTM_TW_SET_UPPER_BITS_MIN_RANGE || parameter > DTM_TW_SET_UPPER_BITS_MAX_RANGE) {
    return DTM_TW_EVENT_TEST_STATUS_ERROR;
  }

  dtm_hci_parameters.test_data_length = (parameter << DTM_TW_UPPER_BITS_POS) & DTM_TW_UPPER_BITS_MASK;
  return DTM_TW_EVENT_TEST_STATUS_SUCCESS;
}

static uint16_t set_phy(uint8_t parameter)
{
  switch (parameter) {
  case DTM_TW_PHY_1M_MIN_RANGE ... DTM_TW_PHY_1M_MAX_RANGE:
    dtm_hci_parameters.tx_phy = BT_HCI_LE_TX_PHY_1M;
    dtm_hci_parameters.rx_phy = BT_HCI_LE_RX_PHY_1M;
    break;
  case DTM_TW_PHY_2M_MIN_RANGE ... DTM_TW_PHY_2M_MAX_RANGE:
    dtm_hci_parameters.tx_phy = BT_HCI_LE_TX_PHY_2M;
    dtm_hci_parameters.rx_phy = BT_HCI_LE_RX_PHY_2M;
    break;
  case DTM_TW_PHY_LE_CODED_S8_MIN_RANGE ... DTM_TW_PHY_LE_CODED_S8_MAX_RANGE:
    dtm_hci_parameters.tx_phy = BT_HCI_LE_TX_PHY_CODED_S8;
    dtm_hci_parameters.rx_phy = BT_HCI_LE_RX_PHY_CODED;
    break;
  case DTM_TW_PHY_LE_CODED_S2_MIN_RANGE ... DTM_TW_PHY_LE_CODED_S2_MAX_RANGE:
    dtm_hci_parameters.tx_phy = BT_HCI_LE_TX_PHY_CODED_S2;
    dtm_hci_parameters.rx_phy = BT_HCI_LE_RX_PHY_CODED;
    break;
  default:
    return DTM_TW_EVENT_TEST_STATUS_ERROR;
  }

  return DTM_TW_EVENT_TEST_STATUS_SUCCESS;
}

static uint16_t set_modulation_index(uint8_t parameter)
{
  switch (parameter) {
  case DTM_TW_MODULATION_INDEX_STANDARD_MIN_RANGE ... DTM_TW_MODULATION_INDEX_STANDARD_MAX_RANGE:
    dtm_hci_parameters.modulation_index = BT_HCI_LE_MOD_INDEX_STANDARD;
    break;
  case DTM_TW_MODULATION_INDEX_STABLE_MIN_RANGE ... DTM_TW_MODULATION_INDEX_STABLE_MAX_RANGE:
    dtm_hci_parameters.modulation_index = BT_HCI_LE_MOD_INDEX_STABLE;
    break;
  default:
    return DTM_TW_EVENT_TEST_STATUS_ERROR;
  }

  return DTM_TW_EVENT_TEST_STATUS_SUCCESS;
}

static uint16_t read_supported_features(uint8_t parameter, struct net_buf *hci_cmd)
{
  if (parameter < DTM_TW_FEATURE_READ_MIN_RANGE || parameter > DTM_TW_FEATURE_READ_MAX_RANGE) {
    return DTM_TW_EVENT_TEST_STATUS_ERROR;
  }

  /* Generate HCI command to read local features */
  struct bt_hci_cmd_hdr *cmd_hdr = net_buf_add(hci_cmd, BT_HCI_CMD_HDR_SIZE);
  cmd_hdr->opcode = BT_HCI_OP_LE_READ_ALL_LOCAL_SUPPORTED_FEATURES;
  cmd_hdr->param_len = 0;
  return HCI_GENERATED;
}

static uint16_t read_max_value(uint8_t parameter, struct net_buf *hci_cmd)
{
  struct bt_hci_cmd_hdr *cmd_hdr = net_buf_add(hci_cmd, BT_HCI_CMD_HDR_SIZE);

  switch (parameter) {
  case DTM_TW_SUPPORTED_TX_OCTETS_MIN_RANGE ... DTM_TW_SUPPORTED_TX_OCTETS_MAX_RANGE:
  case DTM_TW_SUPPORTED_TX_TIME_MIN_RANGE ... DTM_TW_SUPPORTED_TX_TIME_MAX_RANGE:
  case DTM_TW_SUPPORTED_RX_OCTETS_MIN_RANGE ... DTM_TW_SUPPORTED_RX_OCTETS_MAX_RANGE:
  case DTM_TW_SUPPORTED_RX_TIME_MIN_RANGE ... DTM_TW_SUPPORTED_RX_TIME_MAX_RANGE:
    cmd_hdr->opcode = BT_HCI_OP_LE_READ_MAX_DATA_LEN;
    cmd_hdr->param_len = 0;
    break;
  case DTM_TW_SUPPORTED_CTE_LENGTH:
    cmd_hdr->opcode = BT_HCI_OP_LE_READ_ANT_INFO;
    cmd_hdr->param_len = 0;
    break;
  default:
    return DTM_TW_EVENT_TEST_STATUS_ERROR;
  }

  return HCI_GENERATED;
}

static uint16_t set_cte(uint8_t parameter)
{
  /* TODO: DRGN-27910 add CTE and antenna switching support */
  ARG_UNUSED(parameter);
  return DTM_TW_EVENT_TEST_STATUS_ERROR;
}

static uint16_t set_cte_slot(uint8_t parameter)
{
  /* TODO: DRGN-27910 add CTE and antenna switching support */
  ARG_UNUSED(parameter);
  return DTM_TW_EVENT_TEST_STATUS_ERROR;
}

static uint16_t set_antenna_pattern(uint8_t parameter)
{
  /* TODO: DRGN-27910 add CTE and antenna switching support */
  ARG_UNUSED(parameter);
  return DTM_TW_EVENT_TEST_STATUS_ERROR;
}

static uint16_t set_tx_power(int8_t parameter, struct net_buf *hci_cmd)
{
  dtm_hci_parameters.transmit_power = parameter;

  /* Generate HCI command to read min/max TX power levels */
  struct bt_hci_cmd_hdr *cmd_hdr = net_buf_add(hci_cmd, BT_HCI_CMD_HDR_SIZE);
  cmd_hdr->opcode = BT_HCI_OP_LE_READ_TX_POWER;
  cmd_hdr->param_len = 0;
  return HCI_GENERATED;
}

static uint16_t on_test_setup_cmd(enum dtm_tw_ctrl_code control, uint8_t parameter, struct net_buf *hci_cmd)
{
  switch (control) {
  case DTM_TW_TEST_SETUP_RESET:
    return reset_dtm(parameter);

  case DTM_TW_TEST_SETUP_SET_UPPER:
    return set_upper_length_bits(parameter);

  case DTM_TW_TEST_SETUP_SET_PHY:
    return set_phy(parameter);

  case DTM_TW_TEST_SETUP_SELECT_MODULATION:
    return set_modulation_index(parameter);

  case DTM_TW_TEST_SETUP_READ_SUPPORTED:
    return read_supported_features(parameter, hci_cmd);

  case DTM_TW_TEST_SETUP_READ_MAX:
    return read_max_value(parameter, hci_cmd);

  case DTM_TW_TEST_SETUP_CONSTANT_TONE_EXTENSION:
    return set_cte(parameter);

  case DTM_TW_TEST_SETUP_CONSTANT_TONE_EXTENSION_SLOT:
    return set_cte_slot(parameter);

  case DTM_TW_TEST_SETUP_ANTENNA_ARRAY:
    return set_antenna_pattern(parameter);

  case DTM_TW_TEST_SETUP_TRANSMIT_POWER:
    return set_tx_power(parameter, hci_cmd);

  default:
    return DTM_TW_EVENT_TEST_STATUS_ERROR;
  }
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

static uint16_t on_cc_supported_features(const struct bt_hci_rp_le_read_all_local_supported_features *features_rp)
{
  if (features_rp->status != BT_HCI_ERR_SUCCESS) {
    LOG_ERR("Read Local Supported Features command failed with status 0x%02X", features_rp->status);
    return DTM_TW_EVENT_TEST_STATUS_ERROR;
  }

  uint16_t event = DTM_TW_EVENT_TEST_STATUS_SUCCESS;
  event |= (FEATURE_DLE(features_rp->features) ? DTM_TW_TEST_SETUP_DLE_SUPPORTED : 0);
  event |= (FEATURE_2M_PHY(features_rp->features) ? DTM_TW_TEST_SETUP_2M_PHY_SUPPORTED : 0);
  event |= (FEATURE_TX_STABLE_MOD(features_rp->features) ? DTM_TW_TEST_SETUP_STABLE_MODULATION_SUPPORTED : 0);
  event |= (FEATURE_CODED_PHY(features_rp->features) ? DTM_TW_TEST_SETUP_CODED_PHY_SUPPORTED : 0);
  event |= (FEATURE_CTE(features_rp->features) ? DTM_TW_TEST_SETUP_CTE_SUPPORTED : 0);
  event |= (FEATURE_ANT_SWITCHING(features_rp->features) ? DTM_TW_TEST_SETUP_ANTENNA_SWITCH : 0);
  event |= (FEATURE_AOD_1US_TX(features_rp->features) ? DTM_TW_TEST_SETUP_AOD_1US_TX : 0);
  event |= (FEATURE_AOD_1US_RX(features_rp->features) ? DTM_TW_TEST_SETUP_AOD_1US_RX : 0);
  event |= (FEATURE_AOA_1US_RX(features_rp->features) ? DTM_TW_TEST_SETUP_AOA_1US_RX : 0);
  return event;
}

static uint16_t on_cc_read_max(const struct bt_hci_rp_le_read_max_data_len *max_data_len_rp, const uint16_t tw_cmd)
{
  if (max_data_len_rp->status != BT_HCI_ERR_SUCCESS) {
    LOG_ERR("Read Max Data Length command failed with status 0x%02X", max_data_len_rp->status);
    return DTM_TW_EVENT_TEST_STATUS_ERROR;
  }
  /* The max data length event encodes both RX and TX octets and time, so we need to check which parameter the two-wire
   * command was querying for and return the appropriate value. */
  uint16_t ret;
  switch (TW_CMD_PARAMETER(tw_cmd)) {
  case DTM_TW_SUPPORTED_TX_OCTETS_MIN_RANGE ... DTM_TW_SUPPORTED_TX_OCTETS_MAX_RANGE:
    ret = max_data_len_rp->max_tx_octets;
    break;
  case DTM_TW_SUPPORTED_TX_TIME_MIN_RANGE ... DTM_TW_SUPPORTED_TX_TIME_MAX_RANGE:
    /* The HCI command returns the max TX time in microseconds,
     * while the two-wire event returns the max TX time in microseconds divided by 2. */
    ret = max_data_len_rp->max_tx_time >> 1;
    break;
  case DTM_TW_SUPPORTED_RX_OCTETS_MIN_RANGE ... DTM_TW_SUPPORTED_RX_OCTETS_MAX_RANGE:
    ret = max_data_len_rp->max_rx_octets;
    break;
  case DTM_TW_SUPPORTED_RX_TIME_MIN_RANGE ... DTM_TW_SUPPORTED_RX_TIME_MAX_RANGE:
    /* The HCI command returns the max RX time in microseconds,
     * while the two-wire event returns the max RX time in microseconds divided by 2. */
    ret = max_data_len_rp->max_rx_time >> 1;
    break;
  default:
    return DTM_TW_EVENT_TEST_STATUS_ERROR;
  }

  return DTM_TW_EVENT_TEST_STATUS_SUCCESS | ((ret << DTM_TW_EVENT_RESPONSE_POS) & DTM_TW_EVENT_RESPONSE_MASK);
}

static uint16_t on_cc_read_ant_info(const struct bt_hci_rp_le_read_ant_info *ant_info_rp)
{
  if (ant_info_rp->status != BT_HCI_ERR_SUCCESS) {
    LOG_ERR("Read Antenna Information command failed with status 0x%02X", ant_info_rp->status);
    return DTM_TW_EVENT_TEST_STATUS_ERROR;
  }

  return DTM_TW_EVENT_TEST_STATUS_SUCCESS | ((ant_info_rp->max_cte_len << DTM_TW_EVENT_RESPONSE_POS) &
                                             DTM_TW_EVENT_RESPONSE_MASK);
}

static uint16_t on_cc_read_tx_power(const struct bt_hci_rp_le_read_tx_power *tx_power_rp)
{
  if (tx_power_rp->status != BT_HCI_ERR_SUCCESS) {
    LOG_ERR("Read TX Power command failed with status 0x%02X", tx_power_rp->status);
    return DTM_TW_EVENT_TEST_STATUS_ERROR;
  }

  uint16_t event = DTM_TW_EVENT_TEST_STATUS_SUCCESS;

  if (dtm_hci_parameters.transmit_power < tx_power_rp->min_tx_power ||
      dtm_hci_parameters.transmit_power == DTM_TW_TRANSMIT_POWER_LVL_SET_MIN) {
    dtm_hci_parameters.transmit_power = tx_power_rp->min_tx_power;
    event |= DTM_TW_TRANSMIT_POWER_MIN_LVL_BIT;
  }
  else if (dtm_hci_parameters.transmit_power > tx_power_rp->max_tx_power ||
           dtm_hci_parameters.transmit_power == DTM_TW_TRANSMIT_POWER_LVL_SET_MAX) {
    dtm_hci_parameters.transmit_power = tx_power_rp->max_tx_power;
    event |= DTM_TW_TRANSMIT_POWER_MAX_LVL_BIT;
  }

  /* Since we do not communicate the transmit power level to the controller until the test is started,
   * we have to assume the current value of the transmit_power parameter will be accepted. */
  event |= ((dtm_hci_parameters.transmit_power << DTM_TW_TRANSMIT_POWER_RESPONSE_LVL_POS) &
            DTM_TW_TRANSMIT_POWER_RESPONSE_LVL_MASK);
  return event;
}

static int dtm_tw_to_hci_init(void)
{
  reset_dtm(DTM_TW_RESET_MIN_RANGE);

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
  case BT_HCI_OP_LE_READ_MAX_DATA_LEN:
    const struct bt_hci_rp_le_read_max_data_len *max_data_len_rp = cc_event_return_params;
    *tw_event = on_cc_read_max(max_data_len_rp, tw_cmd);
    return DTM_TW_TO_HCI_STATUS_TW_EVENT;

  case BT_HCI_OP_LE_READ_ANT_INFO:
    const struct bt_hci_rp_le_read_ant_info *ant_info_rp = cc_event_return_params;
    *tw_event = on_cc_read_ant_info(ant_info_rp);
    return DTM_TW_TO_HCI_STATUS_TW_EVENT;

  case BT_HCI_OP_LE_READ_ALL_LOCAL_SUPPORTED_FEATURES:
    const struct bt_hci_rp_le_read_all_local_supported_features *features_rp = cc_event_return_params;
    *tw_event = on_cc_supported_features(features_rp);
    return DTM_TW_TO_HCI_STATUS_TW_EVENT;

  case BT_HCI_OP_LE_READ_TX_POWER:
    const struct bt_hci_rp_le_read_tx_power *tx_power_rp = cc_event_return_params;
    *tw_event = on_cc_read_tx_power(tx_power_rp);
    return DTM_TW_TO_HCI_STATUS_TW_EVENT;

  /* TODO: Handle specific DTM-related HCI command complete events, put the resulting two-wire event in tw_event,
   * and return DTM_TW_TO_HCI_STATUS_TW_EVENT. */
  default:
    LOG_WRN("Unhandled HCI command complete event for opcode 0x%04X", cc_event->opcode);
    return DTM_TW_TO_HCI_STATUS_UNHANDLED;
  }
}

SYS_INIT(dtm_tw_to_hci_init, APPLICATION, 0);
