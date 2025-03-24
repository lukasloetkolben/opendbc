import numpy as np
from numpy import clip

from opendbc.car.tesla.values import CarControllerParams
from openpilot.selfdrive.controls.lib.longcontrol import LongControl

DT_CTRL = 0.04
TESTING = True


class TeslaTrafficLight:
  MAX_STOP_LINE_DIST = 127  # maximum valid distance for stop line
  TRAFFIC_LIGHT_REASON = 3  # reason code for traffic light
  RED_LIGHT_STATE = 1
  GREEN_LIGHT_STATE = 2
  YELLOW_LIGHT_STATE = 3
  GREEN_LIGHT_ACCEL = 0.4  # gentle acceleration for green lights

  def __init__(self, CP):
    self.CP = CP
    self.target_speed = 25 / 3.6  # 25 km/h
    self.phase = 0

    self.LoC = LongControl(self.CP)
    self.LoC.pid.i_rate = 0.04
    self.last_accel = 0

  def calculate_required_deceleration(self, v_ego, distance_to_traffic_light):
    target_distance = distance_to_traffic_light - 7

    # If we're already at or past the target point, return a strong deceleration
    if target_distance <= 0:
      return CarControllerParams.ACCEL_MIN

    # Using kinematics equation: a = (vf^2 - vi^2) / (2*d)
    return (self.target_speed ** 2 - v_ego ** 2) / (2 * target_distance)

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
    light_status = self._get_traffic_light_status(CS)

    # Default to current accel
    result_accel = accel

    # If beyond detection range, gas pressed, or ACC not active, just return current accel
    if (not light_status["valid"] or
      light_status["distance"] >= self.MAX_STOP_LINE_DIST or
      gas_pressed or
      not CC.longActive):
      self.phase = 0
      return accel

    # Handle green light case for smooth starts
    if not TESTING and (
      light_status["is_green"] and v_ego < 2 and no_obstacle and CS.das_control["DAS_setSpeed"] > 0 and light_status[
      "distance"] < 4):
      result_accel = max(accel, self.GREEN_LIGHT_ACCEL)
      result_accel = min(result_accel, CS.das_control["DAS_accelMax"])
      self.phase = 0
      return result_accel

    # Reset when conditions change to green
    if not TESTING and light_status["is_green"]:
      self.phase = 0
      return accel

    # Handle yellow light - treat as red if we need significant deceleration
    is_effective_red = TESTING or light_status["is_red"]

    if not TESTING and light_status["is_yellow"]:
      if light_status["distance"] / v_ego >= 3:
        is_effective_red = True

    # Handle red (or effective red) light
    if is_effective_red and ((light_status["distance"] / v_ego) <= 8 or self.phase != 0):
      output_accel = 0
      required_decel = self.calculate_required_deceleration(v_ego, light_status["distance"])
      rate = 0.07

      if required_decel >= -2 and self.phase == 0:
        accel = min(max(required_decel, -0.1), 0.1)
        output_accel = clip(accel, a_ego - rate, a_ego)

      if required_decel < -2 and self.phase == 0:
        self.phase = 1

      if self.phase == 1:
        output_accel = clip(required_decel, self.last_accel - rate, self.last_accel + rate)

      if v_ego <= self.target_speed and self.phase == 1:
        self.phase = 2

      if self.phase == 2:
        accel = min(max(self.target_speed - v_ego, -0.1), 0.1)
        output_accel = clip(accel, self.last_accel - rate, self.last_accel + rate)

      if (light_status["distance"] / v_ego <= 1.5) and self.phase == 2:
        self.phase = 3

      if self.phase == 3:
        rate = self.CP.stoppingDecelRate
        output_accel = clip(-1.95, self.last_accel - rate, self.last_accel + rate)

      if v_ego <= self.CP.vEgoStopping and self.phase == 3:
        self.phase = 4

      if self.phase == 4:
        output_accel = self.CP.stopAccel

      print(f"Phase {self.phase} - {round(accel, 2)}")
      pid_accel_limits = (CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
      required_decel = float(self.LoC.update(CC.longActive, CS.out, output_accel, self.phase == 4, pid_accel_limits))

      # Apply more deceleration when the model is braking, e.g. lead vehicle.
      result_accel = min(accel, required_decel)

    self.last_accel = result_accel
    return result_accel
