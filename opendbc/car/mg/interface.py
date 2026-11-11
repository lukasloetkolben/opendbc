from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mg.carcontroller import CarController
from opendbc.car.mg.carstate import CarState
from opendbc.car.mg.values import CAR


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mg"
    ret.dashcamOnly = True
    ret.radarUnavailable = True

    if candidate == CAR.MG_4_EV:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.noOutput)]
    else:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mg)]

    ret.steerActuatorDelay = 0.3
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    ret.steerControlType = structs.CarParams.SteerControlType.torque

    return ret
