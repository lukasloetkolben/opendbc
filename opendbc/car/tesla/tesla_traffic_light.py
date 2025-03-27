from collections import deque
from numpy import clip
from opendbc.car.tesla.values import CarControllerParams
from openpilot.selfdrive.controls.lib.longcontrol import LongControl

DT_CTRL = 0.04
TESTING = False


class TeslaTrafficLight:
  # Constants
  MAX_STOP_LINE_DIST = 127  # maximum valid distance for stop line
  TRAFFIC_LIGHT_REASON = 3  # reason code for traffic light
  RED_LIGHT_STATE = 1
  GREEN_LIGHT_STATE = 2
  YELLOW_LIGHT_STATE = 3
  GREEN_LIGHT_ACCEL = 0.4  # gentle acceleration for green lights
  SLOW_DOWN_SPEED = 20 / 3.6  # 20 km/h in m/s

  # Phase states
  PHASE_INACTIVE = 0
  PHASE_INITIAL_DECEL = 1
  PHASE_APPROACHING = 3  # Note: Phase 2 is skipped in original code
  PHASE_STOPPING = 4

  def __init__(self, CP):
    self.CP = CP
    self.phase = self.PHASE_INACTIVE
    self.LoC = LongControl(self.CP)
    self.LoC.pid.i_rate = 0.04
    self.last_accel = 0
    self.stop_line_distances = deque([0] * 20, maxlen=20)
    self.required_decelerations = deque([0] * 10, maxlen=10)

  def calculate_required_deceleration(self, v_ego, distance_to_traffic_light, distance_offset, target_speed):
    """Calculate required deceleration based on kinematics"""
    target_distance = distance_to_traffic_light + distance_offset

    # If we're already at or past the target point, return a strong deceleration
    if target_distance <= 0:
      return CarControllerParams.ACCEL_MIN

    # Using kinematics equation: a = (vf^2 - vi^2) / (2*d)
    return (target_speed ** 2 - v_ego ** 2) / (2 * target_distance)

  def _get_traffic_light_status(self, CS):
    """Extract traffic light status information from car state"""
    return {
      "distance": CS.das_road["StopLineDist"],
      "is_red": CS.das_road["TrafficLightState"] == self.RED_LIGHT_STATE,
      "is_yellow": CS.das_road["TrafficLightState"] == self.YELLOW_LIGHT_STATE,
      "is_green": CS.das_road["TrafficLightState"] == self.GREEN_LIGHT_STATE
    }

  def _is_traffic_light_valid(self, avg_stop_line_distance, CS):
    """Check if traffic light detection is valid"""
    if TESTING:
      return avg_stop_line_distance < self.MAX_STOP_LINE_DIST

    return (avg_stop_line_distance < self.MAX_STOP_LINE_DIST and
            CS.das_road["StopLineReason"] == self.TRAFFIC_LIGHT_REASON)

  def _handle_green_light(self, accel, CS, light_status, no_obstacle, v_ego):
    """Handle green light case for smooth starts"""
    if TESTING:
      return None

    if (light_status["is_green"] and
        v_ego < 2 and
        no_obstacle and
        CS.das_control["DAS_setSpeed"] > 0 and
        light_status["distance"] < 4):
      result_accel = max(accel, self.GREEN_LIGHT_ACCEL)
      result_accel = min(result_accel, CS.das_control["DAS_accelMax"])
      return result_accel

    if light_status["is_green"]:
      return accel

    return None

  def _handle_yellow_light(self, light_status, avg_stop_line_distance, v_ego):
    """Determine if yellow light should be treated as red"""
    if TESTING:
      return False

    yellow = light_status["is_yellow"]  and not light_status["is_red"] and not light_status["is_green"]
    if yellow and avg_stop_line_distance < self.MAX_STOP_LINE_DIST:
      # Treat as red if we have enough time to stop comfortably
      return avg_stop_line_distance / v_ego >= 3

    return False

  def _calculate_deceleration_profile(self, v_ego, stop_line_distance, avg_stop_line_distance, accel, a_ego):
    """Calculate deceleration profile based on current phase"""
    if self.phase == self.PHASE_INACTIVE:
      # Initial phase - gentle deceleration
      output_accel = clip(min(accel, -0.5), a_ego - 0.08, a_ego + 0.08)

      # Transition to next phase if we're getting close and moving fast
      if stop_line_distance / v_ego < 6 and v_ego > self.SLOW_DOWN_SPEED:
        self.phase = self.PHASE_INITIAL_DECEL

    elif self.phase == self.PHASE_INITIAL_DECEL:
      # Use average of required decelerations for smoother braking
      output_accel = sum(self.required_decelerations) / len(self.required_decelerations)
      output_accel = clip(output_accel, self.last_accel - 0.08, self.last_accel + 0.08)

      # Transition to approaching phase when we get closer
      if avg_stop_line_distance < 25:
        self.phase = self.PHASE_APPROACHING

    elif self.phase == self.PHASE_APPROACHING:
      # Calculate required deceleration for stopping at the line
      offset = -2
      target_speed = 0
      required_decel = self.calculate_required_deceleration(v_ego, stop_line_distance, offset, target_speed)
      output_accel = required_decel

      # Transition to stopping phase when very close or already stopped
      if v_ego <= self.CP.vEgoStopping or stop_line_distance <= 1:
        self.phase = self.PHASE_STOPPING

    elif self.phase == self.PHASE_STOPPING:
      # Use the configured stopping acceleration
      output_accel = self.CP.stopAccel

    else:
      output_accel = accel

    return output_accel

  def update(self, CC, CS, accel):
    """Update the traffic light controller"""
    # Extract current state
    v_ego = CS.out.vEgo
    a_ego = CS.out.aEgo
    no_obstacle = CS.das_status2["DAS_pmmObstacleSeverity"] == 0
    gas_pressed = CS.out.gasPressed

    # Get traffic light status
    light_status = self._get_traffic_light_status(CS)
    stop_line_distance = light_status["distance"]

    # Update moving averages
    self.stop_line_distances.append(stop_line_distance)
    avg_stop_line_distance = sum(self.stop_line_distances) / len(self.stop_line_distances)

    # Check if traffic light is valid
    valid = self._is_traffic_light_valid(avg_stop_line_distance, CS)

    # Default to current accel
    result_accel = accel

    # If beyond detection range, gas pressed, or ACC not active, just return current accel
    if (not valid or
        avg_stop_line_distance >= self.MAX_STOP_LINE_DIST or
        gas_pressed or
        not CC.longActive):
      self.phase = self.PHASE_INACTIVE
      return accel

    # Handle green light case
    green_light_accel = self._handle_green_light(accel, CS, light_status, no_obstacle, v_ego)
    if green_light_accel is not None:
      self.phase = self.PHASE_INACTIVE
      return green_light_accel

    # Determine if we should treat this as a red light
    is_effective_red = (TESTING or light_status["is_red"] or self._handle_yellow_light(light_status, avg_stop_line_distance, v_ego))

    # Handle red (or effective red) light
    if is_effective_red and ((avg_stop_line_distance / v_ego <= 8) or self.phase != self.PHASE_INACTIVE):
      # Calculate and store required deceleration
      if self.phase == self.PHASE_APPROACHING:
        offset = -2
        target_speed = 0
        distance = stop_line_distance
      else:
        offset = -10
        target_speed = min(CS.out.cruiseState.speed, self.SLOW_DOWN_SPEED)
        distance = avg_stop_line_distance

      required_decel = self.calculate_required_deceleration(v_ego, distance, offset, target_speed)
      self.required_decelerations.append(required_decel)

      # Calculate appropriate deceleration based on current phase
      output_accel = self._calculate_deceleration_profile(v_ego, stop_line_distance, avg_stop_line_distance, accel,
                                                          a_ego)

      # For debugging
      print(f"Phase {self.phase} - {round(accel, 2)}")

      # Apply PID control for smoother control
      pid_accel_limits = (CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
      is_stopping = self.phase == self.PHASE_STOPPING
      required_decel = float(self.LoC.update(CC.longActive, CS.out, output_accel, is_stopping, pid_accel_limits))

      # Apply more deceleration when the model is braking, e.g. lead vehicle
      result_accel = min(accel, required_decel)
    else:
      self.phase = self.PHASE_INACTIVE

    self.last_accel = result_accel
    return result_accel