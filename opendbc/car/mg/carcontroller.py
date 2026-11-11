import numpy as np

from opendbc.can.packer import CANPacker
from opendbc.car import Bus, apply_hysteresis, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits, apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mg.mgcan import (create_lka_steering, create_mg4_lka_steering, create_mg4_acc_cmd,
                                  create_mg4_acc_hold, create_mg4_acc_resume, create_mg4_acc_passthrough, mg4_acc_radar_braking,
                                  create_mg4_cruise_buttons_blank, create_mg4_acc_state)
from opendbc.car.mg.values import CarControllerParams, MgFlags

LongCtrlState = structs.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])

    self.apply_torque_last = 0
    self.apply_angle_last = 0.0
    self.apply_accel_last = 0.0
    self.accel_steady = 0.0

    # ACC_CMD
    self.set_speed_last = 0
    self.decel_regime = False
    self.sng_state = "drive"   # drive | hold | resume
    self.sng_resume_frames = 0

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
    actuators = CC.actuators
    can_sends = []
    P = CarControllerParams

    CS.openpilot_enabled = CC.enabled

    self.apply_angle_last = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                         CS.out.steeringAngleDeg, CC.latActive, P.MG4_ANGLE_LIMITS)
    authority = P.MG4_STEER_AUTHORITY if CC.latActive else 0
    can_sends.append(create_mg4_lka_steering(self.packer, self.frame % 15, self.apply_angle_last, CC.latActive, authority, -authority, 0))

    # forward ACC_STATE with the hands-on warning cleared
    if CS.acc_state_new:
      can_sends.append(create_mg4_acc_state(self.packer, CS.acc_state, 0))

    if self.CP.openpilotLongitudinalControl:
      # blank buttons to the radar so the stock ACC never engages
      if CS.buttons_new:
        can_sends.append(create_mg4_cruise_buttons_blank(self.packer, CS.buttons_counter, 2))

      # ACC_CMD, once per radar frame
      if CS.acc_cmd_new:
        can_sends.append(self.update_acc_cmd(CC, CS))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    new_actuators.accel = self.apply_accel_last

    self.frame += 1
    return new_actuators, can_sends

  def update_acc_cmd(self, CC, CS):
    P = CarControllerParams
    actuators = CC.actuators
    hud = CC.hudControl

    set_speed = int(round(hud.setSpeed * 3.6))
    if 0 < set_speed < 250:
      self.set_speed_last = set_speed
    gap_bars = hud.leadDistanceBars

    # forward the radar's own frame while it asks for braking
    active = CC.longActive and not CS.out.gasPressed and not mg4_acc_radar_braking(CS.acc_cmd)

    # stop-and-go: hold at standstill, resume pulse when openpilot accelerates
    stopping = actuators.longControlState == LongCtrlState.stopping
    if not active:
      self.sng_state = "drive"
    elif self.sng_state == "drive" and stopping and CS.out.vEgoRaw < P.MG4_HOLD_SPEED:
      self.sng_state = "hold"
    elif self.sng_state == "hold" and (CC.cruiseControl.resume or actuators.accel > 0.05):
      self.sng_state = "resume"
      self.sng_resume_frames = P.MG4_RESUME_FRAMES
      self.decel_regime = False
    elif self.sng_state == "resume":
      self.sng_resume_frames -= 1
      if self.sng_resume_frames <= 0:
        self.sng_state = "drive"

    if not active:
      # not controlling (disengaged, gas override, radar braking): the radar's own frame
      self.apply_accel_last = 0.0
      self.decel_regime = False
      return create_mg4_acc_passthrough(self.packer, CS.acc_cmd, self.set_speed_last, gap_bars, 0)

    if self.sng_state == "hold":
      self.apply_accel_last = 0.0
      return create_mg4_acc_hold(self.packer, CS.acc_cmd, set_speed, gap_bars, 0)
    if self.sng_state == "resume":
      self.apply_accel_last = 0.0
      return create_mg4_acc_resume(self.packer, CS.acc_cmd, set_speed, gap_bars, 0)

    # the VCU follows every AccTorqueReq count, so dithering within one count is held
    accel = float(np.clip(actuators.accel, P.MG4_ACCEL_MIN, P.MG4_ACCEL_MAX))
    self.accel_steady = apply_hysteresis(accel, self.accel_steady, P.MG4_ACCEL_HYST_GAP)
    accel = self.accel_steady

    # decel regime below MG4_BRAKE_ENTER, left above MG4_BRAKE_EXIT
    self.decel_regime = accel < (P.MG4_BRAKE_EXIT if self.decel_regime else P.MG4_BRAKE_ENTER)

    self.apply_accel_last = accel
    return create_mg4_acc_cmd(self.packer, CS.acc_cmd, accel, self.decel_regime, set_speed, hud.leadVisible, gap_bars,
                              CS.out.vEgoRaw, 0)
