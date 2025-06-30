import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.rivian.riviancan import create_angle_steering, create_acm_status, create_longitudinal, create_wheel_touch, create_adas_status, create_lka_steering
from opendbc.car.rivian.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])

    self.cancel_frames = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    self.apply_angle_last = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                         CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)

    can_sends.append(create_angle_steering(self.packer, self.frame, self.apply_angle_last, CC.latActive))
    can_sends.append(create_lka_steering(self.packer, self.frame, CS.acm_lka_hba_cmd, CC.enabled))
    can_sends.append(create_acm_status(self.packer, self.frame, CC.enabled))

    if self.frame % 5 == 0:
      can_sends.append(create_wheel_touch(self.packer, CS.sccm_wheel_touch, CC.enabled))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      can_sends.append(create_longitudinal(self.packer, self.frame, accel, CC.enabled))
    else:
      interface_status = None
      if CC.cruiseControl.cancel:
        # if there is a noEntry, we need to send a status of "available" before the ACM will accept "unavailable"
        # send "available" right away as the VDM itself takes a few frames to acknowledge
        interface_status = 1 if self.cancel_frames < 5 else 0
        self.cancel_frames += 1
      else:
        self.cancel_frames = 0

      can_sends.append(create_adas_status(self.packer, CS.vdm_adas_status, interface_status))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
