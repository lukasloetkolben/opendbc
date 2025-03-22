import numpy as np
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CarControllerParams

TESTING = True

class TeslaTrafficLight:
  MAX_STOP_LINE_DIST = 127  # maximum valid distance for stop line
  TRAFFIC_LIGHT_REASON = 3  # reason code for traffic light
  RED_LIGHT_STATE = 1
  GREEN_LIGHT_STATE = 2
  YELLOW_LIGHT_STATE = 3
  YELLOW_DECEL_THRESHOLD = -2.5  # threshold to treat yellow as red
  GREEN_LIGHT_ACCEL = 0.4  # gentle acceleration for green lights

  def __init__(self, CP):
    self.CP = CP
    self.phase = 0
    self.last_accel = 0
    self.phase_one_accel = -2.5
    self.LoC = LongControl(self.CP)
    self.LoC.pid.i_rate = 0.04

  def reset_red_light(self):
    self.phase = 0
    self.phase_one_accel = -2.5

  def calculate_required_deceleration(self, vego, distance_to_stop_m):
    if distance_to_stop_m <= 0:
      return CarControllerParams.ACCEL_MIN

    # a = -v_i²/(2*d)
    distance_based_decel = (vego ** 2) / (2 * distance_to_stop_m)

    return -distance_based_decel

  def _get_traffic_light_status(self, CS):
    """Extract traffic light status information from car state"""
    # Check if stop line is valid and for a traffic light
    stop_line_valid = (CS.das_road["StopLineDist"] < self.MAX_STOP_LINE_DIST and
                       CS.das_road["StopLineReason"] == self.TRAFFIC_LIGHT_REASON)
    if TESTING:
      stop_line_valid = CS.das_road["StopLineDist"] < self.MAX_STOP_LINE_DIST

    if not stop_line_valid:
      return {
        "valid": False,
        "distance": float('inf'),
        "is_red": False,
        "is_yellow": False,
        "is_green": False
      }

    # Get traffic light status
    light_state = CS.das_road["TrafficLightState"]
    distance = CS.das_road["StopLineDist"]

    return {
      "valid": True,
      "distance": distance,
      "is_red": light_state == self.RED_LIGHT_STATE,
      "is_yellow": light_state == self.YELLOW_LIGHT_STATE,
      "is_green": light_state == self.GREEN_LIGHT_STATE
    }

  def update(self, CC, CS, accel):
    """Update the traffic light controller"""
    v_ego = CS.out.vEgo
    a_ego = CS.out.aEgo
    no_obstacle = CS.das_status2["DAS_pmmObstacleSeverity"] == 0
    gas_pressed = CS.out.gasPressed
    result_accel = accel
    light_status = self._get_traffic_light_status(CS)

    if not CC.longActive:
      self.LoC.reset()
      self.reset_red_light()

    if (not light_status["valid"] or
      light_status["distance"] >= self.MAX_STOP_LINE_DIST or
      gas_pressed or
      not CC.longActive):
      self.reset_red_light()
      return accel

    # Handle green light case for smooth starts
    if not TESTING and (
      light_status["is_green"] and v_ego < 2 and no_obstacle and CS.das_control["DAS_setSpeed"] > 0 and light_status[
      "distance"] < 4):
      result_accel = max(accel, self.GREEN_LIGHT_ACCEL)
      result_accel = min(result_accel, CS.das_control["DAS_accelMax"])
      self.LoC.reset()
      self.reset_red_light()
      return result_accel

    # Handle yellow light - treat as red if we need significant deceleration
    is_effective_red = TESTING or light_status["is_red"]
    calculated_decel = self.calculate_required_deceleration(v_ego, light_status["distance"])

    if not TESTING and light_status["is_yellow"]:
      if calculated_decel >= self.YELLOW_DECEL_THRESHOLD:
        is_effective_red = True
      else:
        is_effective_red = False
        self.reset_red_light()

    if is_effective_red:
      rate = 0.07
      should_stop = False

      if self.phase == 0 and calculated_decel < -2:
        self.phase = 1

      if self.phase == 1:
        self.phase_one_accel = min(calculated_decel, self.phase_one_accel)
        accel = self.phase_one_accel
        rate = 0.075

      if v_ego <= 20 * CV.KPH_TO_MS and self.phase == 1:
        self.phase = 2

      if self.phase == 2:
        accel = calculated_decel
        rate = 0.035

      if self.phase == 2 and (light_status["distance"] / v_ego) < 1.75:
        self.phase = 3

      if self.phase == 3:
        should_stop = True

      accel = np.clip(accel, self.last_accel - rate, self.last_accel + rate)
      pid_accel_limits = (CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
      pid_accel = float(self.LoC.update(CC.longActive, CS.out, accel, should_stop, pid_accel_limits))

      # Apply more deceleration when the model is braking, e.g. lead vehicle.
      result_accel = min(accel, pid_accel)

    self.last_accel = result_accel
    return result_accel
