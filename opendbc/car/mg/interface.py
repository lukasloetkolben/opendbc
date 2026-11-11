from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mg.carcontroller import CarController
from opendbc.car.mg.carstate import CarState
from opendbc.car.mg.radar_interface import RadarInterface
from opendbc.car.mg.values import CAR

MG_PARAM_CANFD = 1


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mg"
    ret.steerActuatorDelay = 0.3

    if candidate == CAR.MG_4_EV:
      ret.steerControlType = structs.CarParams.SteerControlType.angle
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mg, MG_PARAM_CANFD)]
    else:
      ret.dashcamOnly = True
      ret.radarUnavailable = True
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mg)]
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    return ret
