#pragma once

#include "opendbc/safety/declarations.h"

static uint8_t mg_crc_lut[256];

// MG4 CAN FD angle steering, selected by safetyParam bit 0. MG5 retains torque steering.
const uint16_t MG_PARAM_CANFD = 1U;
static bool mg_canfd = false;

static uint32_t mg4_checksum(const CANPacket_t *msg) {
  uint8_t crc = 0U;
  for (int i = 5; i < 12; i++) {
    crc = mg_crc_lut[crc ^ msg->data[i]];
  }
  return crc;
}

static void mg4_rx_hook(const CANPacket_t *msg) {
  if (msg_matches(msg, 0xABU, 0U)) {
    update_sample(&angle_meas, (int)((msg->data[6] << 8) | msg->data[7]) - 32768);
  }
  if (msg_matches(msg, 0x13EU, 0U)) {
    unsigned int raw = ((msg->data[30] & 0x7FU) << 8) | msg->data[31];
    vehicle_moving = raw > 6U;
    UPDATE_VEHICLE_SPEED(((float)raw / 64.0f) * KPH_TO_MS);
  }
  if (msg_matches(msg, 0x46U, 0U)) {
    brake_pressed = GET_BIT(msg, 237U);
    gas_pressed = msg->data[30] != 0U;
  }
  if (msg_matches(msg, 0x1DAU, 2U)) {
    unsigned int state = msg->data[17] >> 5;
    pcm_cruise_check((state >= 1U) && (state <= 4U));
  }
}

static bool mg4_tx_hook(const CANPacket_t *msg) {
  const AngleSteeringLimits MG4_LIMITS = {
    .max_angle = 1440,
    .angle_deg_to_can = 16,
    .frequency = 100U,
    .angle_rate_up_lookup = {{0., 5., 25.}, {2.5, 1.5, 0.4}},
    .angle_rate_down_lookup = {{0., 5., 25.}, {5.0, 2.0, 0.8}},
  };
  int angle = (int)((msg->data[6] << 8) | msg->data[7]) - 32600;
  unsigned int state = msg->data[5] >> 4;
  bool active = state == 4U;
  int torque_a = (int)((msg->data[16] << 3) | (msg->data[17] >> 5)) - 1024;
  int torque_b = (int)(((msg->data[17] & 0x1FU) << 6) | (msg->data[18] >> 2)) - 1024;
  bool valid = (state == 0U) || active;
  valid &= (msg->data[5] & 0xFU) < 15U;
  valid &= msg->data[4] == mg4_checksum(msg);
  valid &= (msg->data[0] == 2U) && (msg->data[1] == 9U) && (msg->data[2] == 7U) && (msg->data[3] == 8U);
  valid &= (msg->data[8] == 0x80U) && (msg->data[9] == 0x1FU) && (msg->data[10] == 0xFFU) && (msg->data[11] == 0xFFU);
  valid &= (msg->data[12] == 2U) && (msg->data[13] == 9U) && (msg->data[14] == 0x11U) && (msg->data[15] == 8U);
  valid &= (msg->data[18] & 3U) == 0U;
  for (int i = 19; i < 24; i++) {
    valid &= msg->data[i] == 0U;
  }
  // Bound both directional requests independently; inactive commands must request zero torque.
  valid &= !safety_max_limit_check(torque_a, active ? 150 : 0, 0);
  valid &= !safety_max_limit_check(torque_b, 0, active ? -150 : 0);
  bool violation = steer_angle_cmd_checks(angle, active, MG4_LIMITS);
  violation |= safety_max_limit_check(angle, MG4_LIMITS.max_angle, -MG4_LIMITS.max_angle);
  return valid && !violation;
}

static uint32_t mg_get_checksum(const CANPacket_t *msg) {
  if (mg_canfd) {
    return msg->data[4];
  }
  uint32_t checksum = msg->data[7];
  if (msg->addr == 0x1ecU) {
    checksum = ((msg->data[2] & 0x7FU) << 8) | msg->data[3];
  }
  return checksum;
}

static uint32_t mg_compute_checksum(const CANPacket_t *msg) {
  if (mg_canfd) {
    return mg4_checksum(msg);
  }
  uint32_t checksum;
  if (msg->addr == 0x1ecU) {
    uint32_t torque = ((msg->data[0] & 0x7U) << 8) | msg->data[1];
    uint32_t valid = (msg->data[0] >> 3) & 1U;
    uint32_t status = (msg->data[6] >> 4) & 0x7U;
    uint32_t counter = msg->data[0] >> 4;
    checksum = (0U - ((torque << 1) + (status << 12) + valid + counter)) & 0x7FFFU;
  } else {
    checksum = 0xFFU;
    for (int i = 0; i < 7; i++) {
      checksum ^= msg->data[i];
      checksum = mg_crc_lut[checksum];
    }
    checksum ^= 0xFFU;
  }
  return checksum;
}

static uint8_t mg_get_counter(const CANPacket_t *msg) {
  uint8_t counter = 0U;
  if (msg->addr == 0x23cU) {
    counter = (msg->data[4] >> 4) & 0xFU;
  }
  if (msg->addr == 0x1ecU) {
    counter = (msg->data[0] >> 4) & 0xFU;
  }
  if (msg->addr == 0x242U) {
    counter = (msg->data[0] >> 3) & 0xFU;
  }
  if (msg->addr == 0x1b6U) {
    counter = msg->data[6] & 0xFU;
  }
  if (msg->addr == 0xAFU) {
    counter = msg->data[5] >> 4U;
  }
  return counter;
}

static void mg_rx_hook(const CANPacket_t *msg) {
  if (mg_canfd) {
    mg4_rx_hook(msg);
    return;
  }
  // Vehicle speed
  if (msg_matches(msg, 0x23cU, 0U)) {
    float speed = (((msg->data[2] & 0x7FU) << 8) | msg->data[3]) * 0.015625;
    vehicle_moving = speed > 0.0;
    UPDATE_VEHICLE_SPEED(speed * KPH_TO_MS);
  }

  // Gas pressed
  if (msg_matches(msg, 0xafU, 0U)) {
    gas_pressed = msg->data[0] != 0U;
  }

  // Driver torque
  if (msg_matches(msg, 0x1ecU, 0U)) {
    int torque_driver_new = (((msg->data[4] & 0x7U) << 8) | msg->data[5]) - 1024U;
    update_sample(&torque_driver, torque_driver_new);
  }

  // Brake pressed
  if (msg_matches(msg, 0x1b6U, 0U)) {
    brake_pressed = GET_BIT(msg, 10U);
  }

  // Cruise state
  if (msg_matches(msg, 0x242U, 0U)) {
    int cruise_state = (msg->data[5] & 0x38U) >> 3;
    bool cruise_engaged = (cruise_state == 2) ||  // Active
                          (cruise_state == 3);    // Override
    pcm_cruise_check(cruise_engaged);
  }
}

static bool mg_tx_hook(const CANPacket_t *msg) {
  if (mg_canfd) {
    return mg4_tx_hook(msg);
  }
  const TorqueSteeringLimits MG_STEERING_LIMITS = {
    .max_torque = 300,
    .max_rate_up = 6,
    .max_rate_down = 10,
    .max_rt_delta = 125,
    .driver_torque_multiplier = 2,
    .driver_torque_allowance = 100,
    .type = TorqueDriverLimited,
  };

  // The TX list dispatches only steering control (0x1fd, bus 0) to this hook.
  int desired_torque = (((msg->data[0] & 0x7U) << 8) | msg->data[1]) - 1024U;
  bool steer_req = GET_BIT(msg, 35U);
  return !steer_torque_cmd_checks(desired_torque, steer_req, MG_STEERING_LIMITS);
}

static safety_config mg_init(uint16_t param) {
  mg_canfd = GET_FLAG(param, MG_PARAM_CANFD);

  gen_crc_lookup_table_8(0x1DU, mg_crc_lut);

  if (mg_canfd) {
    static const CanMsg MG4_TX_MSGS[] = {{0x89, 0, 24, .check_relay = true}};
    // Only chunk-0 checksums are known. Counters are not yet validated; ACC_STATE has no known CRC.
    static RxCheck mg4_rx_checks[] = {
      {.msg = {{0xAB, 0, 48, .frequency = 100U, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
      {.msg = {{0x13E, 0, 64, .frequency = 50U, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
      {.msg = {{0x46, 0, 48, .frequency = 100U, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
      {.msg = {{0x1DA, 2, 48, .frequency = 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    };
    return BUILD_SAFETY_CFG(mg4_rx_checks, MG4_TX_MSGS);
  }

  static const CanMsg MG_TX_MSGS[] = {{0x1fd, 0, 8, .check_relay = true}};

  static RxCheck mg_rx_checks[] = {
    {.msg = {{0x23c, 0, 8, .frequency = 50U,  .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // SCS_HSC2_FrP19 (speed)
    {.msg = {{0xaf,  0, 8, .frequency = 100U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // GW_HSC2_HCU_FrP00 (gas pedal)
    {.msg = {{0x1ec, 0, 8, .frequency = 50U,  .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // EPS_HSC2_FrP03 (driver torque)
    {.msg = {{0x242, 0, 8, .frequency = 50U,  .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // RADAR_HSC2_FrP00 (cruise state)
    {.msg = {{0x1b6, 0, 8, .frequency = 50U,  .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // EHBS_HSC2_FrP00 (brake pedal)
  };

  safety_config ret;
  SET_RX_CHECKS(mg_rx_checks, ret);
  SET_TX_MSGS(MG_TX_MSGS, ret);
  return ret;
}

const safety_hooks mg_hooks = {
  .init = mg_init,
  .rx = mg_rx_hook,
  .tx = mg_tx_hook,
  .get_checksum = mg_get_checksum,
  .compute_checksum = mg_compute_checksum,
  .get_counter = mg_get_counter,
};
