def byd_checksum(address: int, sig, d: bytearray) -> int:
  return (~sum(d[:7])) & 0xFF


def create_steering_control(packer, apply_angle: float, lat_active: bool, counter: int):
  # Stock saturates the rate limits at ±299 when engaged, 0 when disengaged
  rate_limit = 299 if lat_active else 0
  values = {
    "STEER_REQ": 1 if lat_active else 0,
    "STEER_REQ_ACTIVE_LOW": 0 if lat_active else 1,
    "STEER_ANGLE": apply_angle,
    "ANGLE_RATE_LIMIT_UPPER": rate_limit,
    "ANGLE_RATE_LIMIT_LOWER": -rate_limit,
    "SET_ME_1_1": 1,
    "SET_ME_1_2": 1,
    "SET_ME_FF": 0xFF,
    "SET_ME_F": 0xF,
    "COUNTER": counter,
  }
  return packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)


def create_buttons(packer, cancel: bool):
  values = {
    "SET_ME_1_1": 1,
    "SET_ME_1_2": 1,
    "ACC_ON_BTN": 1 if cancel else 0,
  }
  return packer.make_can_msg("PCM_BUTTONS", 0, values)


def create_lkas_hud(packer, lat_active: bool, counter: int):
  values = {
    # TODO: test STEER_ACTIVE_1_1 to 3 individual
    "STEER_ACTIVE_1_1": 1 if lat_active else 0,
    "STEER_ACTIVE_1_2": 1 if lat_active else 0,
    "STEER_ACTIVE_1_3": 1 if lat_active else 0,
    "STEER_ACTIVE_ACTIVE_LOW": 0 if lat_active else 1,
    "LSS_STATE": 2 if lat_active else 0,
    "SET_ME_1_2": 1,
    "SET_ME_X5F": 0x5F,
    "SET_ME_XFF": 0xFF,
    "COUNTER": counter,
  }
  return packer.make_can_msg("LKAS_HUD_ADAS", 0, values)
