import numpy as np
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
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

  # Jerk-limited braking parameters
  MAX_DECELERATION = 3.0  # m/s² - comfortable max deceleration (positive value)
  MAX_JERK = 2.0  # m/s³ - comfortable max jerk rate (positive value)

  def __init__(self, CP):
    self.CP = CP

    self.LoC = LongControl(self.CP)
    self.LoC.pid.i_rate = 0.04

    # Internal state for jerk-limited control
    self.last_accel_request = 0.0
    self.last_distance = float('inf')
    self.is_braking_active = False
    self.braking_plan = None
    self.plan_time = 0.0
    self.update_rate = 0.01  # Assuming 100Hz control loop

  def _compute_jerk_limited_braking_plan(self, v_ego, a_ego, distance_to_stop):
    """
    Compute an optimal jerk-limited braking plan based on current state.

    Args:
        v_ego: Current velocity (m/s)
        a_ego: Current acceleration (m/s²)
        distance_to_stop: Distance to stopping point (m)

    Returns:
        dict: Braking plan parameters
    """
    # If already stopped or very slow, no need for complex planning
    if v_ego <= 0.1:
      return {
        "type": "stopped",
        "t_total": 0.0
      }

    # Check if stop is feasible with our constraints
    min_distance_needed = 0.5 * (v_ego ** 2) / self.MAX_DECELERATION

    # If we can't stop in time with comfortable deceleration, use maximum available
    if distance_to_stop < min_distance_needed:
      return {
        "type": "emergency",
        "decel": min(self.MAX_DECELERATION * 1.5, CarControllerParams.ACCEL_MIN * -1)
      }

    # Time to ramp between zero and max deceleration
    t_ramp = self.MAX_DECELERATION / self.MAX_JERK

    # Current deceleration state (only consider deceleration)
    current_decel = max(0, -a_ego)

    # Time to ramp up remaining deceleration
    t_phase1_remaining = (self.MAX_DECELERATION - current_decel) / self.MAX_JERK
    t_phase1_remaining = max(0, t_phase1_remaining)

    # Calculate velocity after ramping to max deceleration
    v_after_phase1 = v_ego - (current_decel * t_phase1_remaining) - \
                     (self.MAX_JERK * t_phase1_remaining ** 2) / 2

    # Calculate distance covered during deceleration ramp-up
    d_phase1 = v_ego * t_phase1_remaining - \
               (current_decel * t_phase1_remaining ** 2) / 2 - \
               (self.MAX_JERK * t_phase1_remaining ** 3) / 6

    # Binary search to find optimal constant deceleration phase duration
    def total_distance(t_const):
      # Distance in constant deceleration phase
      d_phase2 = v_after_phase1 * t_const - \
                 (self.MAX_DECELERATION * t_const ** 2) / 2

      # Velocity after constant deceleration phase
      v_after_phase2 = v_after_phase1 - self.MAX_DECELERATION * t_const

      # If we'd stop during constant deceleration, adjust calculations
      if v_after_phase2 < 0:
        t_const_adjusted = v_after_phase1 / self.MAX_DECELERATION
        d_phase2_adjusted = v_after_phase1 * t_const_adjusted - \
                            (self.MAX_DECELERATION * t_const_adjusted ** 2) / 2
        return d_phase1 + d_phase2_adjusted

      # Distance in deceleration ramp-down phase
      d_phase3 = v_after_phase2 * t_ramp + \
                 (self.MAX_DECELERATION * t_ramp ** 2) / 2 - \
                 (self.MAX_JERK * t_ramp ** 3) / 6

      return d_phase1 + d_phase2 + d_phase3

    # Binary search bounds for constant deceleration phase
    t_const_min = 0
    t_const_max = v_after_phase1 / self.MAX_DECELERATION  # Max possible time

    t_const = (t_const_min + t_const_max) / 2
    epsilon = 1e-6

    # Binary search for optimal constant deceleration duration
    iterations = 0
    max_iterations = 20  # Prevent infinite loops

    while abs(total_distance(t_const) - distance_to_stop) > epsilon and iterations < max_iterations:
      iterations += 1

      if total_distance(t_const) > distance_to_stop:
        t_const_max = t_const
      else:
        t_const_min = t_const

      t_const = (t_const_min + t_const_max) / 2

      if t_const_max - t_const_min < epsilon:
        break

    # Calculate velocity after constant deceleration phase
    v_after_phase2 = v_after_phase1 - self.MAX_DECELERATION * t_const

    # Check if we need a deceleration ramp-down phase
    need_phase3 = v_after_phase2 > 0

    if need_phase3:
      # Standard three-phase plan
      t_phase3 = t_ramp
      t_total = t_phase1_remaining + t_const + t_phase3

      return {
        "type": "three_phase",
        "t_phase1_remaining": t_phase1_remaining,
        "t_const": t_const,
        "t_phase3": t_phase3,
        "current_decel": current_decel,
        "t_total": t_total
      }
    else:
      # Two-phase plan (stop during constant deceleration)
      t_const_adjusted = v_after_phase1 / self.MAX_DECELERATION
      t_total = t_phase1_remaining + t_const_adjusted

      return {
        "type": "two_phase",
        "t_phase1_remaining": t_phase1_remaining,
        "t_const": t_const_adjusted,
        "current_decel": current_decel,
        "t_total": t_total
      }

  def _get_acceleration_from_plan(self, t):
    """
    Get the desired acceleration at time t from the current braking plan.

    Args:
        t: Time since the plan started (seconds)

    Returns:
        float: Desired acceleration (m/s²)
    """
    if self.braking_plan is None:
      return 0.0

    if self.braking_plan["type"] == "stopped":
      return 0.0

    if self.braking_plan["type"] == "emergency":
      return -self.braking_plan["decel"]

    plan = self.braking_plan
    current_decel = plan["current_decel"]

    if plan["type"] == "three_phase":
      t_phase1 = plan["t_phase1_remaining"]
      t_phase2 = plan["t_const"]
      t_phase3 = plan["t_phase3"]

      # Phase 1: Ramp up deceleration
      if t <= t_phase1:
        return -current_decel - self.MAX_JERK * t

      # Phase 2: Constant deceleration
      elif t <= t_phase1 + t_phase2:
        return -self.MAX_DECELERATION

      # Phase 3: Ramp down deceleration
      elif t <= t_phase1 + t_phase2 + t_phase3:
        t_in_phase3 = t - t_phase1 - t_phase2
        return -self.MAX_DECELERATION + self.MAX_JERK * t_in_phase3

      # After plan completion
      else:
        return 0.0

    elif plan["type"] == "two_phase":
      t_phase1 = plan["t_phase1_remaining"]
      t_phase2 = plan["t_const"]

      # Phase 1: Ramp up deceleration
      if t <= t_phase1:
        return -current_decel - self.MAX_JERK * t

      # Phase 2: Constant deceleration until stop
      elif t <= t_phase1 + t_phase2:
        return -self.MAX_DECELERATION

      # After plan completion
      else:
        return 0.0

  def _apply_jerk_limiting(self, target_accel):
    """
    Apply jerk limiting to smooth acceleration changes.

    Args:
        target_accel: Target acceleration

    Returns:
        float: Jerk-limited acceleration
    """
    max_change = self.MAX_JERK * self.update_rate
    accel_change = np.clip(
      target_accel - self.last_accel_request,
      -max_change,
      max_change
    )

    limited_accel = self.last_accel_request + accel_change
    self.last_accel_request = limited_accel

    return limited_accel

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
      self.is_braking_active = False
      self.braking_plan = None
      return accel

    # If beyond detection range, gas pressed, or ACC not active, just return current accel
    if (not light_status["valid"] or
      light_status["distance"] >= self.MAX_STOP_LINE_DIST or
      gas_pressed or
      not CC.longActive):
      self.LoC.reset()
      self.is_braking_active = False
      self.braking_plan = None
      return accel

    # Handle green light case for smooth starts
    if not TESTING and (
      light_status["is_green"] and v_ego < 2 and no_obstacle and
      CS.das_control["DAS_setSpeed"] > 0 and light_status["distance"] < 4):
      result_accel = max(accel, self.GREEN_LIGHT_ACCEL)
      result_accel = min(result_accel, CS.das_control["DAS_accelMax"])
      self.LoC.reset()
      self.is_braking_active = False
      self.braking_plan = None
      return result_accel

    # Handle yellow light - treat as red if we need significant deceleration
    is_effective_red = TESTING or light_status["is_red"]

    # For yellow lights, use the jerk-limited approach to calculate if we should stop
    if not TESTING and light_status["is_yellow"]:
      temp_plan = self._compute_jerk_limited_braking_plan(v_ego, a_ego, light_status["distance"])

      # If plan is emergency or deceleration is beyond threshold, treat as red
      is_emergency = temp_plan.get("type") == "emergency"
      if is_emergency or (temp_plan.get("type") in ["two_phase", "three_phase"] and
                          self.MAX_DECELERATION >= abs(self.YELLOW_DECEL_THRESHOLD)):
        is_effective_red = True

    # Apply jerk-limited braking control for red lights
    if is_effective_red and light_status["distance"] < 50 and v_ego > 0.1:
      # Check if we need to create/update our braking plan
      should_update_plan = (
        not self.is_braking_active or
        self.braking_plan is None or
        abs(light_status["distance"] - self.last_distance) > 2.0  # Distance changed significantly
      )

      if should_update_plan:
        # Create a new braking plan
        self.braking_plan = self._compute_jerk_limited_braking_plan(
          v_ego, a_ego, light_status["distance"]
        )
        self.plan_time = 0.0
        self.is_braking_active = True
        self.last_distance = light_status["distance"]

      # Get acceleration from plan
      accel_request = self._get_acceleration_from_plan(self.plan_time)
      self.plan_time += self.update_rate

      # Apply jerk limiting for smooth transitions
      accel_request = self._apply_jerk_limiting(accel_request)

      # Apply more deceleration when the model is braking (e.g., for lead vehicle)
      result_accel = min(accel, accel_request)

      # For final approach when very close to stop line, use PID controller for precision
      if light_status["distance"] < 5:
        should_stop = light_status["distance"] < 3
        pid_accel_limits = (CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

        # Target velocity of 0 when close to stop line
        target_decel = 0.0
        if not should_stop:
          target_decel = -0.5  # Very gentle deceleration for creeping

        pid_accel = float(self.LoC.update(CC.longActive, CS.out, target_decel, should_stop, pid_accel_limits))

        # Smoothly transition from jerk-limited profile to PID control
        blend_factor = min(1.0, (5.0 - light_status["distance"]) / 2.0)
        result_accel = (1 - blend_factor) * result_accel + blend_factor * pid_accel
    else:
      # Not actively braking for a red light
      self.is_braking_active = False

    return result_accel
