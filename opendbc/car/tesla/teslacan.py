from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CANBUS, CarControllerParams, TeslaFlags


def get_steer_ctrl_type(flags: int, ctrl_type: int) -> int:
  # Returns the flipped signal value for DAS_steeringControlType on FSD 14
  if flags & TeslaFlags.FSD_14:
    return {1: 2, 2: 1}.get(ctrl_type, ctrl_type)
  else:
    return ctrl_type


class TeslaCAN:
  def __init__(self, CP, packer, radar_packer):
    self.CP = CP
    self.packer = packer
    self.yaw_rate_filtered = 0.0
    self.radar_packer = radar_packer

  @staticmethod
  def checksum(msg_id, dat):
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_steering_control(self, angle, enabled, control_type):
    # On FSD 14+, ANGLE_CONTROL behavior changed to allow user winddown while actuating.
    # with openpilot, after overriding w/ ANGLE_CONTROL the wheel snaps back to the original angle abruptly
    # so we now use LANE_KEEP_ASSIST to match stock FSD.
    # see carstate.py for more details
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": get_steer_ctrl_type(self.CP.flags, control_type if enabled else 0),
    }

    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, active):
    from opendbc.car.interfaces import V_CRUISE_MAX

    set_speed = max(v_ego * CV.MS_TO_KPH, 0)
    if active:
      # TODO: this causes jerking after gas override when above set speed
      set_speed = 0 if accel < 0 else V_CRUISE_MAX

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": counter,
    }
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)

  def create_steering_allowed(self):
    values = {
      "APS_eacAllow": 1,
    }

    return self.packer.make_can_msg("APS_eacMonitor", CANBUS.party, values)

  def create_speed_information(self, counter, cs):
    gear = cs.gearShifter

    GEAR_MAP = {
      "unknown": 0,
      "park": 1,
      "reverse": 2,
      "neutral": 3,
      "drive": 4,
    }

    if cs.standstill:
      direction = 2  # standstill
    else:
      direction = 0 if cs.vEgo > 0 else 1  # 0=forward 1=backward

    values = {
      "Speed": cs.vEgo,
      "Gear": GEAR_MAP[gear],
      "Active": 1,
      "wheelDirectionFrL": direction,
      "vehicleStopping": 1 if cs.standstill else 0,
      "wheelDirectionFrR": direction,
      "wheelDirectionReL": direction,
      "wheelDirectionReR": direction,
      "Counter": counter % 16,
      "Checksum": 0
    }

    data = self.radar_packer.make_can_msg("SpeedInformation", 1, values)[1]
    values["Checksum"] = self.checksum(0x50, data[:-1])
    return self.radar_packer.make_can_msg("SpeedInformation", 1, values)

  def create_radar_yaw_rate(self, counter, cs):
      # Low-pass filter yaw rate to avoid noisy LSBs triggering vehDynamicsError
      # Reference car changes by max ±1 raw count per message (0.0002 deg/s step)
      alpha = 0.15  # filter coefficient (lower = smoother)
      self.yaw_rate_filtered += alpha * (cs.yawRate * CV.RAD_TO_DEG - self.yaw_rate_filtered)

      values = {
              "Acceleration": cs.aEgo,
              "YawRate": self.yaw_rate_filtered,
              "Counter": counter % 16,
              "SETME_15": 15,
              "SETME_3": 3,
              "Checksum": 0,
      }

      data = self.radar_packer.make_can_msg("YawRateInformation", 1, values)[1]
      values["Checksum"] = self.checksum(0x51, data[:-1])
      return self.radar_packer.make_can_msg("YawRateInformation", 1, values)

  def create_speed_information2(self, counter, cs):
    values = {
      "wheelSpeedFrL": cs.vEgo * 3.6,
      "wheelSpeedFrR": cs.vEgo * 3.6,
      "wheelSpeedReL": cs.vEgo * 3.6,
      "wheelSpeedReR": cs.vEgo * 3.6,
      "Counter": counter % 16,
      "Checksum": 0
    }

    data = self.radar_packer.make_can_msg("SpeedInformation2", 1, values)[1]
    values["Checksum"] = self.checksum(0x52, data[:-1])
    return self.radar_packer.make_can_msg("SpeedInformation2", 1, values)

  def create_radar_lateral_information(self, counter, cs):
      values = {
              "steeringWheelAngle": -cs.steeringAngleDeg,
              "steeringAngleSpeed": -cs.steeringRateDeg,
              "Counter": counter % 16,
              "Checksum": 0,
      }

      data = self.radar_packer.make_can_msg("LateralInformation", 1, values)[1]
      values["Checksum"] = self.checksum(0x53, data[:-1])
      return self.radar_packer.make_can_msg("LateralInformation", 1, values)


def tesla_checksum(address: int, sig, d: bytearray) -> int:
  checksum = (address & 0xFF) + ((address >> 8) & 0xFF)
  checksum_byte = sig.start_bit // 8
  for i in range(len(d)):
    if i != checksum_byte:
      checksum += d[i]
  return checksum & 0xFF
