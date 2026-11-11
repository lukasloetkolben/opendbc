from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mg.carcontroller import CarController
from opendbc.car.mg.carstate import CarState
from opendbc.car.mg.radar_interface import RadarInterface
from opendbc.car.mg.values import CAR, CarControllerParams

MG_PARAM_CANFD = 1
MG_PARAM_LONGITUDINAL = 2


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.MG4_ACCEL_MIN, CarControllerParams.MG4_ACCEL_MAX

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mg"
    ret.steerActuatorDelay = 0.3

    if candidate == CAR.MG_4_EV:
      ret.steerControlType = structs.CarParams.SteerControlType.angle

      ret.alphaLongitudinalAvailable = True
      ret.openpilotLongitudinalControl = alpha_long
      ret.pcmCruise = not ret.openpilotLongitudinalControl
      ret.longitudinalTuning.kiV = [0.15]

      safety_param = MG_PARAM_CANFD | (MG_PARAM_LONGITUDINAL if ret.openpilotLongitudinalControl else 0)
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mg, safety_param)]
    else:
      ret.dashcamOnly = True
      ret.radarUnavailable = True
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mg)]
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    return ret
