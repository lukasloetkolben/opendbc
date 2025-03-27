from collections import deque

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
  SLOW_DOWN_SPEED = 20 / 3.6  # 25km/h

  def __init__(self, CP):
    self.CP = CP
    self.phase = 0
    self.LoC = LongControl(self.CP)
    self.LoC.pid.i_rate = 0.04
    self.last_accel = 0
    self.stop_line_distances = deque([0] * 20, maxlen=20)
    self.required_decelerations = deque([0] * 25, maxlen=25)

  def calculate_required_deceleration(self, v_ego, distance_to_traffic_light, target_speed=0, offset = 0):
    target_distance = distance_to_traffic_light + offset

    # If we're already at or past the target point, return a strong deceleration
    if target_distance <= 0:
      return CarControllerParams.ACCEL_MIN

    # Using kinematics equation: a = (vf^2 - vi^2) / (2*d)
    return (target_speed ** 2 - v_ego ** 2) / (2 * target_distance)

  def update(self, CC, CS, accel):
    """Update the traffic light controller"""
    # Extract traffic light status directly
    light_state = CS.das_road["TrafficLightState"]
    stop_line_distance = CS.das_road["StopLineDist"]
    is_red = light_state == self.RED_LIGHT_STATE
    is_yellow = light_state == self.YELLOW_LIGHT_STATE
    is_green = light_state == self.GREEN_LIGHT_STATE

    # Get vehicle state
    v_ego = CS.out.vEgo
    no_obstacle = CS.das_status2["DAS_pmmObstacleSeverity"] == 0
    gas_pressed = CS.out.gasPressed

    # Update distance tracking
    self.stop_line_distances.append(stop_line_distance)
    avg_stop_line_distance = sum(self.stop_line_distances) / len(self.stop_line_distances)
    valid = (avg_stop_line_distance < self.MAX_STOP_LINE_DIST and
             CS.das_road["StopLineReason"] == self.TRAFFIC_LIGHT_REASON)

    if TESTING:
      valid = avg_stop_line_distance < self.MAX_STOP_LINE_DIST

    # Default to current accel
    result_accel = accel

    # If beyond detection range, gas pressed, or ACC not active, just return current accel
    if (not valid or
        avg_stop_line_distance >= self.MAX_STOP_LINE_DIST or
        gas_pressed or
        not CC.longActive):
      self.phase = 0
      return accel

    # Handle green light case for smooth starts
    if not TESTING and (is_green and v_ego < 2 and no_obstacle and
                        CS.das_control["DAS_setSpeed"] > 0 and stop_line_distance < 4):
      result_accel = max(accel, self.GREEN_LIGHT_ACCEL)
      result_accel = min(result_accel, CS.das_control["DAS_accelMax"])
      self.phase = 0
      return result_accel

    # Reset when conditions change to green
    if not TESTING and is_green:
      self.phase = 0
      return accel

    # Handle yellow light - treat as red if we need significant deceleration
    is_effective_red = (TESTING or is_red) and valid

    if not TESTING and (is_yellow and valid):
      if avg_stop_line_distance / v_ego >= 3:
        is_effective_red = True

    # Handle red (or effective red) light
    time = np.interp(v_ego, [20, 60], [3, 5])
    if is_effective_red and ((avg_stop_line_distance / v_ego) <= time or self.phase != 0):
      if self.phase == 3:
        distance = stop_line_distance
        offset = 0
      else:
        distance = avg_stop_line_distance
        offset = -2

      required_decel = self.calculate_required_deceleration(v_ego, distance, offset)
      self.required_decelerations.append(required_decel)
      output_accel = 0

      if self.phase == 0:
        output_accel =  np.mean(list(self.required_decelerations))
        output_accel = clip(output_accel, self.last_accel - 0.08, self.last_accel + 0.08)

      if avg_stop_line_distance < 12 and self.phase == 0:
        self.phase = 3

      if self.phase == 3:
        output_accel = np.mean(list(self.required_decelerations)[-3:])
        output_accel = clip(output_accel, self.last_accel - 0.08, self.last_accel + 0.08)

      if (v_ego <= self.CP.vEgoStopping or stop_line_distance <= 1.5) and self.phase == 3:
        self.phase = 4

      if self.phase == 4:
        output_accel = self.CP.stopAccel

      print(f"Phase {self.phase} - {round(accel, 2)}")
      pid_accel_limits = (CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
      required_decel = float(self.LoC.update(CC.longActive, CS.out, output_accel, self.phase == 4, pid_accel_limits))

      # Apply more deceleration when the model is braking, e.g. lead vehicle.
      result_accel = min(accel, required_decel)
    else:
      self.phase = 0

    self.last_accel = result_accel
    return result_accel
