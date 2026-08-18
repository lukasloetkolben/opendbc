from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mg.carcontroller import CarController
from opendbc.car.mg.carstate import CarState
from opendbc.car.mg.values import CAR

# allOutput passthrough: relay open, panda forwards bus 0 <-> bus 2 (see safety/modes/defaults.h)
ALLOUTPUT_PARAM_PASSTHROUGH = 1


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mg"
    ret.dashcamOnly = True

    if candidate == CAR.MG_4_EV:
      # passthrough mode for reverse-engineering: split the camera and car buses
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.allOutput, ALLOUTPUT_PARAM_PASSTHROUGH)]
    else:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mg)]

    ret.steerActuatorDelay = 0.3
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.radarUnavailable = True
    ret.alphaLongitudinalAvailable = False

    return ret
