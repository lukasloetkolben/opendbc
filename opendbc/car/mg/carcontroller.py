import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mg.mgcan import create_lka_steering
from opendbc.car.mg.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])

    self.cancel_frames = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    apply_torque = 0

    if CC.latActive:
      apply_torque = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))

    # send steering command
    if self.frame % 2 == 0:
      self.apply_torque_last = apply_torque
      can_sends.append(create_lka_steering(self.packer, (self.frame // 2) % 16, apply_torque, CC.latActive))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    return new_actuators, can_sends
