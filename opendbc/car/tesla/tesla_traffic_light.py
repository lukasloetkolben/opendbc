import numpy as np
from numpy import clip

from opendbc.car.tesla.values import CarControllerParams
from openpilot.common.pid import PIDController
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_safe_obstacle_distance, \
  get_stopped_equivalence_factor, CRASH_DISTANCE

DT_CTRL = 0.04
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
    self.stopping = False

    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             k_f=CP.longitudinalTuning.kf, rate=1 / DT_CTRL)

    self.pid.neg_limit = CarControllerParams.ACCEL_MIN
    self.pid.pos_limit = CarControllerParams.ACCEL_MAX

    self.last_result_accel = 0

  def calculate_required_deceleration(self, aego, vego, distance_to_stop_m):
    """
    Calculate the deceleration required to stop at a traffic light by modeling it
    as a stopped vehicle at the stop line position.

    Parameters:
    aego (float): Current ego acceleration in m/s^2
    vego (float): Current ego velocity in m/s
    distance_to_stop_m (float): Distance to the stop line in meters

    Returns:
    float: Required deceleration in m/s^2 to safely stop
    """
    # If already stopped or very slow, no need to decelerate
    if vego < 0.1:
      return 0.0

    # Calibration factor to adjust stopping distance
    # Decrease this value to stop closer to the stop line (e.g., 0.7-0.9)
    # Increase this value to stop further from the stop line (e.g., 1.1-1.3)
    STOP_DISTANCE_SCALE = 0.7  # Calibration factor

    # Apply the calibration factor to the distance
    calibrated_distance = distance_to_stop_m * STOP_DISTANCE_SCALE

    # Target distance from stop line when fully stopped
    FINAL_DISTANCE = 1.0  # 1 meter from the stop line when stopped

    # Handle immediate danger case (too close to stop line)
    if calibrated_distance < CRASH_DISTANCE:
      return CarControllerParams.ACCEL_MIN

    # Basic kinematic calculation for required deceleration
    # Using v_f^2 = v_i^2 + 2*a*d, where v_f = 0 (we want to stop)
    # Solving for a: a = -v_i^2 / (2*d)
    # We subtract FINAL_DISTANCE to stop with our target gap
    stopping_distance = max(0.1, calibrated_distance - FINAL_DISTANCE)

    # Calculate basic required deceleration to stop with the desired gap
    if stopping_distance > 0:
      required_decel = -(vego ** 2) / (2 * stopping_distance)
    else:
      # If we're already at or past our target stopping point, maximum braking
      required_decel = CarControllerParams.ACCEL_MIN

    # Add a margin of safety by increasing the magnitude (more negative)
    # This makes the car start braking a bit earlier than strictly necessary
    safety_margin = 1.1  # 10% more deceleration than the basic physics calculation
    required_decel = required_decel * safety_margin

    # Calculate minimum safe braking distance with current velocity
    # This is a physics-based minimum stopping distance
    min_safe_distance = (vego ** 2) / (2 * abs(CarControllerParams.ACCEL_MIN))

    # If we're too close to stop comfortably, increase braking force
    if calibrated_distance < min_safe_distance + FINAL_DISTANCE:
      # Calculate how much we've violated the minimum safe distance
      violation_ratio = 1.0 - (calibrated_distance / (min_safe_distance + FINAL_DISTANCE))
      violation_ratio = clip(violation_ratio, 0.0, 1.0)

      # Blend between calculated deceleration and maximum deceleration
      # based on how much we've violated the safe distance
      required_decel = required_decel * (1.0 - violation_ratio) + CarControllerParams.ACCEL_MIN * violation_ratio

    # Ensure deceleration is within vehicle capabilities and never positive (no acceleration)
    required_decel = clip(required_decel, CarControllerParams.ACCEL_MIN, 0.0)

    # Limit acceleration change rate (jerk) for smoother braking
    max_accel_change = 2.0  # m/s³ * dt
    prev_decel = min(aego, 0.0)  # Only consider braking component
    required_decel = clip(required_decel,
                          prev_decel - max_accel_change,
                          prev_decel + max_accel_change)

    return required_decel

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
      self.stopping = False
      self.pid.reset()
      return accel

    # Handle green light case for smooth starts
    if not TESTING and (
      light_status["is_green"] and v_ego < 2 and no_obstacle and CS.das_control["DAS_setSpeed"] > 0 and light_status[
      "distance"] < 4):
      result_accel = max(accel, self.GREEN_LIGHT_ACCEL)
      result_accel = min(result_accel, CS.das_control["DAS_accelMax"])
      self.stopping = False
      self.pid.reset()
      return result_accel

    # Reset when conditions change to green
    if not TESTING and light_status["is_green"]:
      self.stopping = False
      self.pid.reset()
      return accel

    # Handle yellow light - treat as red if we need significant deceleration
    is_effective_red = light_status["is_red"]
    if TESTING:
      is_effective_red = light_status["valid"]

    required_decel = self.calculate_required_deceleration(a_ego, v_ego, light_status["distance"])

    if not TESTING and light_status["is_yellow"]:
      if required_decel >= self.YELLOW_DECEL_THRESHOLD:
        is_effective_red = True

    # Handle red (or effective red) light
    if is_effective_red:

      error = self.last_result_accel - a_ego
      output_accel = self.pid.update(error, speed=v_ego, feedforward=required_decel)

      # Apply more deceleration when the model is braking, e.g. lead vehicle.
      result_accel = min(accel, output_accel)


    self.last_result_accel = result_accel
    return result_accel
