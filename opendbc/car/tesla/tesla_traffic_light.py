import numpy as np
from numpy import clip

from opendbc.car.tesla.values import CarControllerParams
from openpilot.common.realtime import DT_CTRL
from openpilot.common.pid import PIDController
from openpilot.common.filter_simple import FirstOrderFilter


class TeslaTrafficLight:

  MAX_STOP_LINE_DIST = 127  # maximum valid distance for stop line
  TRAFFIC_LIGHT_REASON = 3  # reason code for traffic light
  RED_LIGHT_STATE = 1
  GREEN_LIGHT_STATE = 2
  YELLOW_LIGHT_STATE = 3

  # Distance thresholds
  DETECTION_RANGE = 125  # maximum range to consider traffic lights
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

    self.decel_filter = FirstOrderFilter(0.0, 2.0, DT_CTRL)

  def calc_stopping_distance(self, v_ego):
    """Calculate the distance needed to stop from current speed"""
    return (v_ego ** 2) / (2 * abs(CarControllerParams.ACCEL_MIN))

  def calculate_required_deceleration(self, vego, distance_to_stop_m):
    if vego <= self.CP.vEgoStopping:
      return self.CP.stopAccel  # Already stopped

    # a = -v_i²/(2*d)
    required_deceleration = (vego ** 2) / (2 * distance_to_stop_m)

    # come to full standstill
    if distance_to_stop_m <= 5:
      a_stop = float(np.interp(distance_to_stop_m, [5, 0], [-1.75, CarControllerParams.ACCEL_MIN]))
      return min(-required_deceleration, a_stop)

    return -required_deceleration

  def _get_traffic_light_status(self, CS):
    """Extract traffic light status information from car state"""
    # Check if stop line is valid and for a traffic light
    stop_line_valid = (CS.das_road["StopLineDist"] < self.MAX_STOP_LINE_DIST and
                       CS.das_road["StopLineReason"] == self.TRAFFIC_LIGHT_REASON)

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
    no_obstacle = CS.das_status2["DAS_pmmObstacleSeverity"] == 0
    gas_pressed = CS.out.gasPressed

    light_status = self._get_traffic_light_status(CS)

    # Default to current accel
    result_accel = accel

    # If beyond detection range, gas pressed, or ACC not active, just return current accel
    if (not light_status["valid"] or
      light_status["distance"] > self.DETECTION_RANGE or
      gas_pressed or
      not CC.longActive):
      self.stopping = False
      self.pid.reset()
      self.decel_filter.update(accel)
      return accel

    # Handle green light case for smooth starts
    if (light_status["is_green"] and v_ego < 2 and no_obstacle and CS.das_control["DAS_setSpeed"] > 0 and light_status[
      "distance"] < 4):
      result_accel = max(accel, self.GREEN_LIGHT_ACCEL)
      result_accel = min(result_accel, CS.das_control["DAS_accelMax"])
      self.stopping = False
      self.pid.reset()
      self.decel_filter.update(result_accel)
      return result_accel

    # Reset when conditions change to green
    if light_status["is_green"]:
      self.stopping = False
      self.pid.reset()
      self.decel_filter.update(accel)
      return accel

    # Handle yellow light - treat as red if we need significant deceleration
    is_effective_red = light_status["is_red"]
    required_decel = self.calculate_required_deceleration(v_ego, light_status["distance"])

    if light_status["is_yellow"]:
      if required_decel >= self.YELLOW_DECEL_THRESHOLD:
        is_effective_red = True

    # Handle red (or effective red) light
    if is_effective_red:

      if required_decel > -1.85 and not self.stopping:
          output_accel = 0.0
      else:
        # Active stopping mode
        self.stopping = True

        error = required_decel - CS.aEgo
        output_accel = self.pid.update(error, speed=CS.vEgo, feedforward=required_decel)

      # Smooth deceleration
      required_decel = self.decel_filter.update(output_accel)
      # Apply more deceleration when the model is braking, e.g. lead vehicle.
      result_accel = min(accel, required_decel)
    else:
      # For non-red/yellow lights, update the filter with current accel
      self.decel_filter.update(accel)

    return result_accel
