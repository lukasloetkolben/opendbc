import copy
from opendbc.car.common.numpy_fast import clip
from opendbc.can.packer import CANPacker
from opendbc.car import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.rivian.riviancan import create_steering, create_longitudinal, create_vdm_adas_status
from opendbc.car.rivian.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    self.CP = CP
    self.frame = 0
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_name)
    self.engaged = False

  def update(self, CC, CS, now_nanos):

    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel

    can_sends = []

    if CC.latActive:
      apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgo, CarControllerParams)
      apply_angle = clip(apply_angle, CS.out.steeringAngleDeg - 20, CS.out.steeringAngleDeg + 20)
    else:
      apply_angle = CS.out.steeringAngleDeg

    self.apply_angle_last = apply_angle
    can_sends.append(create_steering(self.packer, (CS.steering_control_counter + 1) % 15, apply_angle, CC.latActive))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      accel = CS.longitudinal_request["ACM_AccelerationRequest"]

      if CC.longActive:
        self.engaged = True

      if CS.out.brakePressed:
        self.engaged = False

      can_sends.append(create_longitudinal(self.packer, (CS.longitudinal_request_counter + 1) % 15, accel, self.engaged))


    new_actuators = copy.copy(actuators)
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
