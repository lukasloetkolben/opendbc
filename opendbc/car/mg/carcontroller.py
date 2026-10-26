from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits, apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mg.mgcan import create_lka_steering, create_mg4_lka_steering
from opendbc.car.mg.values import CarControllerParams, MgFlags


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])

    self.apply_torque_last = 0
    self.apply_angle_last = 0.0

  def update(self, CC, CS, now_nanos):
    if self.CP.flags & MgFlags.CANFD:
      return self.update_mg4(CC, CS, now_nanos)

    actuators = CC.actuators
    can_sends = []

    # steering command
    if self.frame % CarControllerParams.STEER_STEP == 0:
      if CC.latActive:
        new_torque = int(round(actuators.torque * CarControllerParams.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, CarControllerParams)
      else:
        apply_torque = 0

      self.apply_torque_last = apply_torque
      can_sends.append(create_lka_steering(self.packer, (self.frame // CarControllerParams.STEER_STEP) % 16, apply_torque, CC.latActive))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends

  def update_mg4(self, CC, CS, now_nanos):
    # MG4: angle-based LKA_CMD (0x089) at 100Hz. The steering angle setpoint (bytes 6-7) is ours;
    # the counter + CRC8 are recomputed; bytes 16-18 (LKAReqToq + LKACmdPV) are copied live from the
    # camera's own 0x089 (see CarState), on the hypothesis that the EPS does not validate the PV.
    actuators = CC.actuators
    can_sends = []
    P = CarControllerParams

    if self.frame % P.MG4_STEER_STEP == 0:
      apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last,
                                                 CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                 CC.latActive, P.MG4_ANGLE_LIMITS)
      self.apply_angle_last = apply_angle

      counter = (self.frame // P.MG4_STEER_STEP) % 15  # camera cycles 0..14
      can_sends.append(create_mg4_lka_steering(counter, apply_angle, CC.latActive,
                                               CS.lka_req_toq, CS.lka_cmd_pv, 0))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
