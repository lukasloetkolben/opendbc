from opendbc.car.interfaces import V_CRUISE_MAX
from opendbc.car.tesla.values import CANBUS, CarControllerParams


class TeslaCAN:
  def __init__(self, packer):
    self.packer = packer

  @staticmethod
  def checksum(msg_id, dat):
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_steering_control(self, angle, enabled, counter):
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,
      "DAS_steeringControlCounter": counter,
    }

    data = self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)[1]
    values["DAS_steeringControlChecksum"] = self.checksum(0x488, data[:3])
    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, cntr, active):
    values = {
      "DAS_setSpeed": 0 if (accel < 0 or not active) else V_CRUISE_MAX,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": cntr,
      "DAS_controlChecksum": 0,
    }
    data = self.packer.make_can_msg("DAS_control", CANBUS.party, values)[1]
    values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)

  def create_steering_allowed(self, counter):
    values = {
      "APS_eacAllow": 1,
      "APS_eacMonitorCounter": counter,
    }

    data = self.packer.make_can_msg("APS_eacMonitor", CANBUS.party, values)[1]
    values["APS_eacMonitorChecksum"] = self.checksum(0x27d, data[:2])
    return self.packer.make_can_msg("APS_eacMonitor", CANBUS.party, values)

  def create_das_status(self, das_status, cntr, active):
    values = {
      "DAS_statusCounter": cntr,
      "DAS_summonAvailable": das_status["DAS_summonAvailable"],
      "DAS_autoLaneChangeState": das_status["DAS_autoLaneChangeState"],
      "DAS_autopilotHandsOnState": das_status["DAS_autopilotHandsOnState"],
      "DAS_fleetSpeedState": das_status["DAS_fleetSpeedState"],
      "DAS_laneDepartureWarning": das_status["DAS_laneDepartureWarning"],
      "DAS_csaState": das_status["DAS_csaState"],
      "DAS_sideCollisionInhibit": das_status["DAS_sideCollisionInhibit"],
      "DAS_sideCollisionWarning": das_status["DAS_sideCollisionWarning"],
      "DAS_sideCollisionAvoid": das_status["DAS_sideCollisionAvoid"],
      "DAS_summonRvsLeashReached": das_status["DAS_summonRvsLeashReached"],
      "DAS_summonFwdLeashReached": das_status["DAS_summonFwdLeashReached"],
      "DAS_autoparkWaitingForBrake": das_status["DAS_autoparkWaitingForBrake"],
      "DAS_autoParked": das_status["DAS_autoParked"],
      "DAS_autoparkReady": das_status["DAS_autoparkReady"],
      "DAS_forwardCollisionWarning": das_status["DAS_forwardCollisionWarning"],
      "DAS_heaterState": das_status["DAS_heaterState"],
      "DAS_visionOnlySpeedLimit": das_status["DAS_visionOnlySpeedLimit"],
      "DAS_summonClearedGate": das_status["DAS_summonClearedGate"],
      "DAS_summonObstacle": das_status["DAS_summonObstacle"],
      "DAS_suppressSpeedWarning": das_status["DAS_suppressSpeedWarning"],
      "DAS_fusedSpeedLimit": das_status["DAS_fusedSpeedLimit"],
      "DAS_blindSpotRearRight": das_status["DAS_blindSpotRearRight"],
      "DAS_blindSpotRearLeft": das_status["DAS_blindSpotRearLeft"],
      "DAS_autopilotState": 3 if active else das_status["DAS_autopilotState"],
    }

    data = self.packer.make_can_msg("DAS_status", CANBUS.party, values)[1]
    values["DAS_statusChecksum"] = self.checksum(0x39B, data[:7])
    return self.packer.make_can_msg("DAS_status", CANBUS.party, values)
