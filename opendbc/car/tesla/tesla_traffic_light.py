import numpy as np
from numpy import clip
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.pid import PIDController
from openpilot.selfdrive.controls.lib.longcontrol import LongControl

from opendbc.car.car_helpers import get_car_interface
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CarControllerParams

TESTING = False

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

    self.CI = get_car_interface(self.CP)
    self.LoC = LongControl(self.CP)
    self.LoC.pid.i_rate = 0.04

  def calculate_required_deceleration(self, vego, distance_to_stop_m):
    # Target time to stop - creates a more comfortable deceleration profile
    # Higher speeds get more time to decelerate gradually
    target_time = max(3.0, vego * 0.45)

    # Progressive deceleration profile based on distance
    # This creates a more natural feeling than constant deceleration
    decel_factor = float(np.interp(distance_to_stop_m, [10, 30], [1.0, 0.6]))

    # Base required deceleration to stop in target time
    # a = -v_f/t where v_f is current velocity (for smooth ramp-down)
    time_based_decel = vego / target_time

    # Distance-based deceleration (physics approach)
    # a = -v_i²/(2*d)
    distance_based_decel = (vego ** 2) / (2 * max(0.1, distance_to_stop_m))

    # Combine both approaches, weighted by distance
    # Far away: rely more on time-based comfort
    # Near stop line: rely more on distance-based precision
    distance_weight = min(1.0, 15.0 / max(1.0, distance_to_stop_m))
    required_deceleration = (1 - distance_weight) * time_based_decel + distance_weight * distance_based_decel

    # Apply deceleration factor to create progressive profile
    required_deceleration *= decel_factor

    # Ensure deceleration is negative (slowing down)
    return -required_deceleration

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

    # If beyond detection range, gas pressed, or ACC not active, just return current accel
    if (not light_status["valid"] or
      light_status["distance"] >= self.MAX_STOP_LINE_DIST or
      gas_pressed or
      not CC.longActive):
      return accel

    # Handle green light case for smooth starts
    if not TESTING and (
      light_status["is_green"] and v_ego < 2 and no_obstacle and CS.das_control["DAS_setSpeed"] > 0 and light_status[
      "distance"] < 4):
      result_accel = max(accel, self.GREEN_LIGHT_ACCEL)
      result_accel = min(result_accel, CS.das_control["DAS_accelMax"])
      self.LoC.reset()
      return result_accel

    # Handle yellow light - treat as red if we need significant deceleration
    is_effective_red = TESTING or light_status["is_red"]
    calculated_decel = self.calculate_required_deceleration(v_ego, light_status["distance"])

    if not TESTING and light_status["is_yellow"]:
      if calculated_decel >= self.YELLOW_DECEL_THRESHOLD:
        is_effective_red = True

    # accel PID loop
    if is_effective_red:
      if calculated_decel > -1.5:
        calculated_decel = np.clip(calculated_decel, a_ego - 0.07, a_ego + 0.07)

      should_stop = light_status["distance"] < 6
      pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
      required_decel = float(self.LoC.update(CC.longActive, CS, calculated_decel, should_stop, pid_accel_limits))

      # Apply more deceleration when the model is braking, e.g. lead vehicle.
      result_accel = min(accel, required_decel)

    return result_accel
