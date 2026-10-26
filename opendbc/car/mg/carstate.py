from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.mg.values import DBC, MgFlags
from opendbc.car.common.conversions import Conversions as CV

GearShifter = structs.CarState.GearShifter

GEAR_MAP = {
  0: GearShifter.unknown,
  15: GearShifter.park,
  14: GearShifter.reverse,
  13: GearShifter.neutral,
  **{i: GearShifter.drive for i in range(1, 9)},
}

GEAR_MAP_CANFD = {
  1: GearShifter.park,
  2: GearShifter.reverse,
  3: GearShifter.neutral,
  4: GearShifter.drive,
}


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    # bytes 16-18 of the camera's LKA_CMD (0x089), replayed in our own command (idle defaults)
    self.lka_req_toq = 0x80
    self.lka_cmd_pv = 0x400

  def update(self, can_parsers) -> structs.CarState:
    if self.CP.flags & MgFlags.CANFD:
      return self.update_canfd(can_parsers)

    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgoRaw = cp.vl["SCS_HSC2_FrP19"]["VehSpdAvgHSC2"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = cp.vl["SCS_HSC2_FrP24"]["VehSdslStsHSC2"] == 1

    # Gas pedal
    ret.gasPressed = cp.vl["GW_HSC2_HCU_FrP00"]["EPTAccelActuPosHSC2"] > 0

    # Brake pedal
    ret.brakePressed = cp.vl["EHBS_HSC2_FrP00"]["BrkPdlAppdHSC2"] == 1

    # Steering wheel
    ret.steeringAngleDeg = cp.vl["SAS_HSC2_FrP00"]["StrgWhlAngHSC2"]
    ret.steeringRateDeg = cp.vl["SAS_HSC2_FrP00"]["StrgWhlAngGrdHSC2"]
    ret.steeringTorque = cp.vl["EPS_HSC2_FrP03"]["DrvrStrgDlvrdToqHSC2"]
    ret.steeringTorqueEps = cp.vl["EPS_HSC2_FrP03"]["ChLKARespToqHSC2"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 1.0, 5)

    ret.steerFaultTemporary = cp_cam.vl["FVCM_HSC2_FrP02"]["LDWSysFltStsHSC2"] != 0

    # Cruise state
    ret.cruiseState.enabled = cp.vl["RADAR_HSC2_FrP00"]["ACCSysSts_RadarHSC2"] in (2, 3)  # Active, Override
    ret.cruiseState.available = cp.vl["RADAR_HSC2_FrP00"]["ACCSysSts_RadarHSC2"] != 0
    ret.cruiseState.standstill = False
    ret.cruiseState.speed = cp.vl["RADAR_HSC2_FrP02"]["ACCDrvrSelTrgtSpd_RadarHSC2"] * CV.KPH_TO_MS

    ret.accFaulted = cp.vl["RADAR_HSC2_FrP00"]["ACCSysFltSts_SCSHSC2"] != 0

    # Gear
    ret.gearShifter = GEAR_MAP.get(int(cp.vl["GW_HSC2_ECM_FrP04"]["TrEstdGearHSC2"]), GearShifter.unknown)

    # Doors
    ret.doorOpen = any([cp.vl["GW_HSC2_BCM_FrP04"]["DrvrDoorOpenStsHSC2"],
                        cp.vl["GW_HSC2_BCM_FrP04"]["FrtPsngDoorOpenStsHSC2"],
                        cp.vl["GW_HSC2_BCM_FrP04"]["RLDoorOpenStsHSC2"],
                        cp.vl["GW_HSC2_BCM_FrP04"]["RRDoorOpenStsHSC2"]])

    # Blinkers
    ret.leftBlinker = cp.vl["GW_HSC2_BCM_FrP04"]["DircnIndLampSwStsHSC2"] == 1
    ret.rightBlinker = cp.vl["GW_HSC2_BCM_FrP04"]["DircnIndLampSwStsHSC2"] == 2

    # Seatbelt
    ret.seatbeltUnlatched = cp.vl["GW_HSC2_SDM_FrP00"]["DrvrSbltAtcHSC2"] != 1

    return ret

  def update_canfd(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    # Copy bytes 16-18 of the camera's LKA_CMD (0x089) so the carcontroller can replay them.
    self.lka_req_toq = int(cp_cam.vl["LKA_CMD"]["LKAReqToq"])
    self.lka_cmd_pv = int(cp_cam.vl["LKA_CMD"]["LKACmdPV"])

    # Vehicle speed
    self.parse_wheel_speeds(ret,
      cp.vl["ESP_ACCEL"]["WhlSpdFL"],
      cp.vl["ESP_ACCEL"]["WhlSpdFR"],
      cp.vl["ESP_ACCEL"]["WhlSpdRL"],
      cp.vl["ESP_ACCEL"]["WhlSpdRR"],
    )
    ret.standstill = cp.vl["ESP_SPEED"]["Standstill"] == 1

    # Gas pedal
    ret.gasPressed = cp.vl["GAS_PEDAL"]["GasPdlPos"] > 0

    # Brake pedal
    ret.brakePressed = cp.vl["BRAKE"]["BrkPdlAppd"] == 1

    # Steering wheel
    ret.steeringAngleDeg = cp.vl["STEER_ANGLE"]["StrgWhlAng"]
    ret.steeringRateDeg = cp.vl["STEER_ANGLE"]["StrgWhlAngRate"]
    # Driver steering torque is not on any captured bus (the EPS handles override internally), so
    # steeringTorque/steeringPressed can't be populated from CAN here.

    # TODO: find fault signals
    # ret.steerFaultTemporary = ...
    # ret.accFaulted = ...

    # Cruise state
    acc_sts = cp.vl["ACC_STATE"]["AccSts"]
    ret.cruiseState.enabled = acc_sts in (2, 3, 4)  # Active, Active+Standby, Override
    ret.cruiseState.available = acc_sts != 0
    ret.cruiseState.standstill = False
    # TODO: set speed not found yet, only shown in cluster?
    # ret.cruiseState.speed = ...

    # Gear
    ret.gearShifter = GEAR_MAP_CANFD.get(int(cp.vl["GEAR"]["ShftLvrPos"]), GearShifter.unknown)

    # Blinkers
    ret.leftBlinker = cp.vl["TURN_SIGNALS"]["TurnIndLeft"] == 1
    ret.rightBlinker = cp.vl["TURN_SIGNALS"]["TurnIndRight"] == 1

    # Seatbelt
    ret.seatbeltUnlatched = cp.vl["ESP_SPEED"]["DrvrSbltUnbuckled"] == 1

    # Doors
    ret.doorOpen = any([cp.vl["DOORS"]["DrvrDoorOpen"],
                        cp.vl["DOORS"]["FrontPsngDoorOpen"],
                        cp.vl["DOORS"]["RearLeftDoorOpen"],
                        cp.vl["DOORS"]["RearRightDoorOpen"]])

    return ret

  @staticmethod
  def get_can_parsers(CP):
    if CP.flags & MgFlags.CANFD:
      return {
        Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
        Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
      }

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
