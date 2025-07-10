#pragma once

#include "opendbc/safety/safety_declarations.h"

static bool tesla_powertrain = false;
static bool tesla_raven_aeb = false;

// Only rising edges while controls are not allowed are considered for these systems:
// TODO: Only LKAS (non-emergency) is currently supported since we've only seen it
static bool tesla_raven_stock_lkas = false;
static bool tesla_raven_stock_lkas_prev = false;

static void tesla_raven_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  // Steering angle: (0.1 * val) - 819.2 in deg.
  if (!tesla_powertrain && bus == 0 && addr == 0x370) {
    // Store it 1/10 deg to match steering request
    const int angle_meas_new = (((GET_BYTE(to_push, 4) & 0x3FU) << 8) | GET_BYTE(to_push, 5)) - 8192U;
    update_sample(&angle_meas, angle_meas_new);

    const int hands_on_level = GET_BYTE(to_push, 4) >> 6;  // EPAS3S_handsOnLevel
    const int eac_status = GET_BYTE(to_push, 6) >> 5;  // EPAS3S_eacStatus
    const int eac_error_code = GET_BYTE(to_push, 2) >> 4;  // EPAS3S_eacErrorCode

    // Disengage on normal user override, or if high angle rate fault from user overriding extremely quickly
    steering_disengage = (hands_on_level >= 3) || ((eac_status == 0) && (eac_error_code == 9));
  }

  // Vehicle speed (DI_torque2: DI_vehicleSpeed)
  if (!tesla_powertrain && bus == 1 && addr == 0x118) {
    // Vehicle speed: ((0.05 * val) - 25) * MPH_TO_MPS
    float speed = (((((GET_BYTE(to_push, 3) & 0x0FU) << 8) | (GET_BYTE(to_push, 2))) * 0.05) - 25) * 0.447;
    UPDATE_VEHICLE_SPEED(speed);
  }

  // Gas pressed
  if (tesla_powertrain && bus == 0 && addr == 0x106) {
    gas_pressed = GET_BYTE(to_push, 6) != 0U;
  }

  if ((tesla_powertrain && (bus == 0) && (addr == 0x1f8)) ||
     (!tesla_powertrain && (bus == 1) && (addr == 0x20a))) {
    brake_pressed = (((GET_BYTE(to_push, 0) & 0x0CU) >> 2) != 1U);
  }

  // Cruise
  if ((tesla_powertrain && (bus == 0) && (addr == 0x256)) ||
     (!tesla_powertrain && (bus == 1) && (addr == 0x368))) {
      // Cruise state
      int cruise_state = (GET_BYTE(to_push, 1) >> 4) & 0x07U;
      bool cruise_engaged = (cruise_state == 2) ||  // ENABLED
                            (cruise_state == 3) ||  // STANDSTILL
                            (cruise_state == 4) ||  // OVERRIDE
                            (cruise_state == 6) ||  // PRE_FAULT
                            (cruise_state == 7);    // PRE_CANCEL
      vehicle_moving = cruise_state != 3; // STANDSTILL
      pcm_cruise_check(cruise_engaged);
   }

  if (bus == 2) {
    // DAS_control
    if (tesla_powertrain && addr == 0x2bf) {
      // "AEB_ACTIVE"
      tesla_raven_aeb = (GET_BYTE(to_push, 2) & 0x03U) == 1U;
    }

    // DAS_steeringControl
    if (!tesla_powertrain && addr == 0x488) {
      int steering_control_type = GET_BYTE(to_push, 2) >> 6;
      bool tesla_raven_stock_lkas_now = steering_control_type == 2;  // "LANE_KEEP_ASSIST"

      // Only consider rising edges while controls are not allowed
      if (tesla_raven_stock_lkas_now && !tesla_raven_stock_lkas_prev && !controls_allowed) {
        tesla_raven_stock_lkas = true;
      }
      if (!tesla_raven_stock_lkas_now) {
        tesla_raven_stock_lkas = false;
      }
      tesla_raven_stock_lkas_prev = tesla_raven_stock_lkas_now;
    }
  }
}


static bool tesla_raven_tx_hook(const CANPacket_t *to_send) {
  const AngleSteeringLimits TESLA_STEERING_LIMITS = {
    .max_angle = 3600,  // 360 deg, EPAS faults above this
    .angle_deg_to_can = 10,
    .frequency = 50U,
  };

  // NOTE: based off TESLA_MODEL_Y to match openpilot
  const AngleSteeringParams TESLA_STEERING_PARAMS = {
    .slip_factor = -0.000580374383851451,  // calc_slip_factor(VM)
    .steer_ratio = 12.,
    .wheelbase = 2.89,
  };

  const LongitudinalLimits TESLA_LONG_LIMITS = {
    .max_accel = 425,       // 2 m/s^2
    .min_accel = 288,       // -3.48 m/s^2
    .inactive_accel = 375,  // 0. m/s^2
  };

  bool tx = true;
  int addr = GET_ADDR(to_send);
  bool violation = false;

  // Steering control: (0.1 * val) - 1638.35 in deg.
  if (!tesla_powertrain && addr == 0x488) {
    // We use 1/10 deg as a unit here
    int raw_angle_can = ((GET_BYTE(to_send, 0) & 0x7FU) << 8) | GET_BYTE(to_send, 1);
    int desired_angle = raw_angle_can - 16384;
    int steer_control_type = GET_BYTE(to_send, 2) >> 6;
    bool steer_control_enabled = steer_control_type == 1;  // ANGLE_CONTROL

    if (steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled, TESLA_STEERING_LIMITS, TESLA_STEERING_PARAMS)) {
      violation = true;
    }

    bool valid_steer_control_type = (steer_control_type == 0) ||  // NONE
                                    (steer_control_type == 1);    // ANGLE_CONTROL
    if (!valid_steer_control_type) {
      violation = true;
    }

    if (tesla_raven_stock_lkas) {
      // Don't allow any steering commands when stock LKAS is active
      violation = true;
    }
  }

  // DAS_control: longitudinal control message
  if (tesla_powertrain && (addr == 0x2bf)) {
    // No AEB events may be sent by openpilot
    int aeb_event = GET_BYTE(to_send, 2) & 0x03U;
    if (aeb_event != 0) {
      violation = true;
    }

    // Don't send long/cancel messages when the stock AEB system is active
    if (tesla_raven_aeb) {
      violation = true;
    }

    int raw_accel_max = ((GET_BYTE(to_send, 6) & 0x1FU) << 4) | (GET_BYTE(to_send, 5) >> 4);
    int raw_accel_min = ((GET_BYTE(to_send, 5) & 0x0FU) << 5) | (GET_BYTE(to_send, 4) >> 3);

    // Prevent both acceleration from being negative, as this could cause the car to reverse after coming to standstill
    if ((raw_accel_max < TESLA_LONG_LIMITS.inactive_accel) && (raw_accel_min < TESLA_LONG_LIMITS.inactive_accel)) {
      violation = true;
    }

    // Don't allow any acceleration limits above the safety limits
    violation |= longitudinal_accel_checks(raw_accel_max, TESLA_LONG_LIMITS);
    violation |= longitudinal_accel_checks(raw_accel_min, TESLA_LONG_LIMITS);

  }

  if (violation) {
    tx = false;
  }

  return tx;
}

static bool tesla_raven_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  if (bus_num == 2) {
    // APS_eacMonitor
    if (!tesla_powertrain && addr == 0x27d) {
      block_msg = true;
    }

    // DAS_steeringControl
    if (!tesla_powertrain && (addr == 0x488) && !tesla_raven_stock_lkas) {
      block_msg = true;
    }

    // DAS_control
    if (tesla_powertrain && (addr == 0x2bf) && !tesla_raven_aeb) {
      block_msg = true;
    }
  }

  return block_msg;
}

static safety_config tesla_raven_init(uint16_t param) {

  static const CanMsg TESLA_TX_RAVEN_MSGS[] = {
    {0x488, 0, 4, .check_relay = true, .disable_static_blocking = true},  // DAS_steeringControl
    {0x27D, 0, 3, .check_relay = true, .disable_static_blocking = true},  // APS_eacMonitor
  };

  static const CanMsg TESLA_RAVEN_PT_MSGS[] = {
    {0x2bf, 0, 8, .check_relay = true, .disable_static_blocking = true},  // DAS_control
  };

  const int TESLA_FLAG_POWERTRAIN = 2;
  tesla_powertrain = GET_FLAG(param, TESLA_FLAG_POWERTRAIN);


  tesla_raven_aeb = false;
  tesla_raven_stock_lkas = false;
  tesla_raven_stock_lkas_prev = false;

  safety_config ret;
  if (tesla_powertrain) {
    static RxCheck tesla_raven_pt_rx_checks[] = {
      {.msg = {{0x106, 0, 8, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true, .frequency = 100U}, { 0 }, { 0 }}},  // DI_torque1
      {.msg = {{0x1f8, 0, 8, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},   // BrakeMessage
      {.msg = {{0x2bf, 2, 8, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true, .frequency = 25U}, { 0 }, { 0 }}},   // DAS_control
      {.msg = {{0x256, 0, 8, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, { 0 }, { 0 }}},   // DI_state
    };
    ret = BUILD_SAFETY_CFG(tesla_raven_pt_rx_checks, TESLA_RAVEN_PT_MSGS);
  } else {
    static RxCheck tesla_raven_rx_checks[] = {
      {.msg = {{0x370, 0, 8, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true, .frequency = 100U}, { 0 }, { 0 }}},  // EPAS3P_sysStatus
      {.msg = {{0x118, 1, 6, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true, .frequency = 100U}, { 0 }, { 0 }}},  // DI_torque2
      {.msg = {{0x20a, 1, 8, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, { 0 }, { 0 }}},   // BrakeMessage
      {.msg = {{0x368, 1, 8, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, { 0 }, { 0 }}},   // DI_state
    };
    ret = BUILD_SAFETY_CFG(tesla_raven_rx_checks, TESLA_TX_RAVEN_MSGS);
  }
  return ret;
}

const safety_hooks tesla_raven_hooks = {
  .init = tesla_raven_init,
  .rx = tesla_raven_rx_hook,
  .tx = tesla_raven_tx_hook,
  .fwd = tesla_raven_fwd_hook,
};
