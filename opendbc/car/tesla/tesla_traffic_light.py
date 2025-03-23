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

    # For a traffic light, we'll treat it as a stationary lead vehicle (v_lead = 0)
    v_lead = 0.0

    # Use the same safety calculations as in long_mpc.py for stopping distance
    # The traffic light acts as a non-moving obstacle (v=0) at distance_to_stop_m
    t_follow = 0.5  # Use a smaller follow time for traffic lights compared to vehicles

    # Calculate the safe following distance using the same formula as in the MPC
    safe_distance = get_safe_obstacle_distance(vego, t_follow)

    # The effective distance we want to maintain from the stop line
    # is the safe distance minus the stopping equivalence factor of the lead (which is 0)
    desired_distance = safe_distance - get_stopped_equivalence_factor(v_lead)

    # If we're closer than the desired distance, we need to brake
    distance_error = distance_to_stop_m - desired_distance

    # Handle immediate danger case
    if distance_to_stop_m < CRASH_DISTANCE:
      return CarControllerParams.ACCEL_MIN

    # If we have sufficient distance, maintain current acceleration
    if distance_error > 0:
      # We're far enough away, just maintain current acceleration with a cap at 0
      # (don't accelerate toward a stop)
      return min(aego, 0.0)

    # Calculate how much deceleration is needed based on the distance error
    # and current velocity (similar to what MPC optimization would do)
    #
    # We want deceleration proportional to how much we've violated the safe distance
    # normalized by the stopping distance

    # First calculate a base deceleration assuming constant deceleration to stop
    # Using the equation: v_f^2 = v_i^2 + 2*a*d
    # For stopping v_f = 0, so a = -v_i^2/(2*d)
    if distance_to_stop_m > 0:
      base_decel = -(vego ** 2) / (2 * max(distance_to_stop_m, 0.1))
    else:
      # If we're at or past the stop line, maximum braking
      base_decel = CarControllerParams.ACCEL_MIN

    # Scale the deceleration based on how much we've violated the safety distance
    # The more we violate, the closer to max braking we want to be
    distance_violation_ratio = min(1.0, abs(distance_error) / safe_distance)

    # Blend between the base deceleration and max deceleration based on violation
    required_decel = base_decel * (
        1.0 - distance_violation_ratio) + CarControllerParams.ACCEL_MIN * distance_violation_ratio

    # Clip to vehicle capabilities and ensure we're not accelerating
    required_decel = clip(required_decel, CarControllerParams.ACCEL_MIN, 0.0)

    # Limit acceleration change rate (jerk) similar to MPC's jerk cost
    # This provides smoother transitions in braking
    max_accel_change = 2.0 * DT_CTRL  # m/s³ * dt
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
