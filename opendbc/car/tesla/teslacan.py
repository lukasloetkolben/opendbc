from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import V_CRUISE_MAX
from opendbc.car.tesla.values import CANBUS, CarControllerParams

class TeslaCAN:
  def __init__(self, packer, radar_packer):
    self.packer = packer
    self.radar_packer = radar_packer

  @staticmethod
  def checksum(msg_id, dat):
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_steering_control(self, angle, enabled):
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,
    }

    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, active):
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
      "wheelDirectionFrR": direction,
      "wheelDirectionReL": direction,
      "wheelDirectionReR": direction,
      "Counter": counter % 16,
      "Checksum": 0
    }

    data = self.radar_packer.make_can_msg("SpeedInformation", CANBUS.party, values)[1]
    values["Checksum"] = self.checksum(0x50, data[:-1])
    return self.radar_packer.make_can_msg("SpeedInformation", 1, values)

  def create_speed_information2(self, counter, cs):
    values = {
      "Speed": cs.vEgo,
      "Counter": counter % 16,
      "Checksum": 0
    }

    data = self.radar_packer.make_can_msg("SpeedInformation2", CANBUS.party, values)[1]
    values["Checksum"] = self.checksum(0x52, data[:-1])
    return self.radar_packer.make_can_msg("SpeedInformation2", 1, values)

  def create_radar_yaw_rate(self, counter, cs):
      values = {
              "Acceleration": cs.aEgo,
              "YawRate": cs.yawRate * CV.RAD_TO_DEG,
              "Counter": counter % 16,
              "UNKOWN": 21,
              "SETME_15": 15,
              "SETME_6": 6,
              "SETME_3": 3,
              "Checksum": 0,
      }

      data = self.radar_packer.make_can_msg("YawRateInformation", CANBUS.party, values)[1]
      values["Checksum"] = self.checksum(0x51, data[:-1])
      return self.radar_packer.make_can_msg("YawRateInformation", 1, values)
