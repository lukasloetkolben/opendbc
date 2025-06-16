import copy
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from opendbc.car import Bus, CanSignalRateCalculator, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.values import DBC, CANBUS, GEAR_MAP, DOORS, STEER_THRESHOLD

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.can_define_party = CANDefine(DBC[CP.carFingerprint][Bus.party])
    self.can_define_pt = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.can_define_chassis = CANDefine(DBC[CP.carFingerprint][Bus.chassis])
    self.can_defines = {
      **self.can_define_party.dv,
      **self.can_define_pt.dv,
      **self.can_define_chassis.dv,
    }

    # TODO: this should be swapped on the harnesses
    CANBUS.chassis = 1
    CANBUS.radar = 5

    self.cruise_enabled_prev = False

    self.hands_on_level = 0
    self.das_control = None
    self.angle_rate_calulator = CanSignalRateCalculator(100)

  def update(self, can_parsers) -> structs.CarState:
    cp_party = can_parsers[Bus.party]        # bus 0
    cp_ap_party = can_parsers[Bus.ap_party]  # bus 2
    cp_pt = can_parsers[Bus.pt]              # bus 4
    cp_ap_pt = can_parsers[Bus.ap_pt]        # bus 6
    cp_chassis = can_parsers[Bus.chassis]    # bus 1
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgoRaw = cp_chassis.vl["DI_torque2"]["DI_vehicleSpeed"] * CV.MPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # Gas pedal
    pedal_status = cp_ap_party.vl["DI_torque1"]["DI_pedalPos"]
    ret.gas = pedal_status / 100.0
    ret.gasPressed = pedal_status > 0

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp_party.vl["IBST_private2"]["IBST_brakePedalApplied"] == 1

    # Steering wheel
    epas_status = cp_party.vl[f"EPAS_sysStatus"]
    self.hands_on_level = epas_status[f"EPAS_handsOnLevel"]
    ret.steeringAngleDeg = -epas_status[f"EPAS_internalSAS"]
    ret.steeringRateDeg = self.angle_rate_calulator.update(ret.steeringAngleDeg, epas_status[f"EPAS_sysStatusCounter"])
    ret.steeringTorque = -epas_status[f"EPAS_torsionBarTorque"]

    # This matches stock logic, but with halved minimum frames (0.25-0.3s)
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 15)
    eac_status = self.can_defines[f"EPAS_sysStatus"][f"EPAS_eacStatus"].get(int(epas_status[f"EPAS_eacStatus"]), None)
    ret.steerFaultPermanent = eac_status == "EAC_FAULT"
    ret.steerFaultTemporary = eac_status == "EAC_INHIBITED"

    # FSD disengages using union of handsOnLevel (slow overrides) and high angle rate faults (fast overrides, high speed)
    # TODO: implement in safety
    eac_error_code = self.can_defines[f"EPAS_sysStatus"][f"EPAS_eacErrorCode"].get(int(epas_status[f"EPAS_eacErrorCode"]), None)
    ret.steeringDisengage = self.hands_on_level >= 3 or (eac_status == "EAC_INHIBITED" and
                                                         eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")

    # Cruise state
    di_state = cp_chassis.vl["DI_state"]

    cruise_state = self.can_defines["DI_state"]["DI_cruiseState"].get(int(di_state["DI_cruiseState"]), None)
    speed_units = self.can_defines["DI_state"]["DI_speedUnits"].get(int(di_state["DI_speedUnits"]), None)

    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")

    # Match panda safety cruise engaged logic
    ret.cruiseState.enabled = cruise_enabled
    if speed_units == "KPH":
      ret.cruiseState.speed = max(di_state["DI_digitalSpeed"] * CV.KPH_TO_MS, 1e-3)
    elif speed_units == "MPH":
      ret.cruiseState.speed = max(di_state["DI_digitalSpeed"] * CV.MPH_TO_MS, 1e-3)
    ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled
    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special
    ret.standstill = cruise_state == "STANDSTILL"
    ret.accFaulted = cruise_state == "FAULT"

    # Gear
    ret.gearShifter = GEAR_MAP.get(int(cp_chassis.vl["DI_torque2"]["DI_gear"]))

    ret.doorOpen = any((self.can_defines["GTW_carState"][door].get(int(cp_chassis.vl["GTW_carState"][door]), "OPEN") == "OPEN") for door in DOORS)
    ret.leftBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorLStatus"] == 1
    ret.rightBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorRStatus"] == 1
    ret.seatbeltUnlatched = cp_chassis.vl["SDM1"]["SDM_bcklDrivStatus"] != 1

    # AEB
    ret.stockAeb = cp_ap_pt.vl["DAS_control"]["DAS_aebEvent"] == 1

    # LKAS
    ret.stockLkas = cp_ap_party.vl["DAS_steeringControl"]["DAS_steeringControlType"] == 2  # LANE_KEEP_ASSIST

    # Messages needed by carcontroller
    self.das_control = copy.copy(cp_ap_pt.vl["DAS_control"])

    return ret

  @staticmethod
  def get_can_parsers(CP):
    party_messages = [
      ("EPAS_sysStatus", 100),
      ("IBST_private2", 50),
    ]

    ap_party_messages = [
      ("DAS_steeringControl", 50),
      ("DI_torque1", 100),
    ]

    pt_messages = [
    ]

    ap_pt_messages = [
      ("DAS_control", 25),
    ]

    chassis_messages = [
      ("DI_torque2", 100),
      ("DI_state", 10),
      ("GTW_carState", 10),
      ("SDM1", 10),
    ]

    parsers = {
      Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], party_messages, CANBUS.party),                  # bus 0
      Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party], ap_party_messages, CANBUS.autopilot_party),  # bus 2
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CANBUS.powertrain),                      # bus 4
      Bus.ap_pt: CANParser(DBC[CP.carFingerprint][Bus.pt], ap_pt_messages, CANBUS.autopilot_powertrain),      # bus 6
      Bus.chassis: CANParser(DBC[CP.carFingerprint][Bus.chassis], chassis_messages, CANBUS.chassis),          # bus 1
    }

    return parsers
