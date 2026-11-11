import numpy as np

from opendbc.car.crc import CRC8J1850
from opendbc.car.mg.values import CarControllerParams

def create_mg4_lka_steering(packer, counter, angle_deg, active, authority_left, authority_right, bus):
  values = {
    "PDU_HEADER_0": 0x02090708,
    "ROLLING_COUNTER": counter,
    "LKACmdSts": 4 if active else 0,
    "LKACmd": angle_deg,
    "SET_ME_X801FFFFF": 0x801FFFFF,
    "PDU_HEADER_1": 0x02091108,
    "LKAReqToqA": authority_left,
    "LKAReqToqB": authority_right,
  }
  return packer.make_can_msg("LKA_CMD", bus, values)


# ACC_CMD: the radar's frame with openpilot's request, unknown fields are copied
ACC_MODE_REGULATING = 4
ACC_MODE_RESUME = 5
ACC_MODE_HOLD = 10
ACC_HOLD_ACCEL = -2.0          # m/s2, stock hold frame
ACC_RESUME_TORQUE_BYTE = 166   # stock resume pulse, torque on near zero


def mg4_torque_byte(accel: float, v_ego: float) -> int:
  P = CarControllerParams
  zero = np.interp(v_ego * 3.6, P.MG4_TORQUE_ZERO_BP, P.MG4_TORQUE_ZERO_V)
  return int(np.clip(round(zero + accel / P.MG4_TORQUE_PER_COUNT), P.MG4_TORQUE_FLOOR, P.MG4_TORQUE_MAX))


def _acc_cmd(packer, acc_cmd, overrides, set_speed_kph, lead, gap_bars, bus):
  values = raw_words(acc_cmd)
  values.update(overrides)
  if set_speed_kph is not None:
    v = int(np.clip(round(set_speed_kph), 0, 255))
    values["AccSetSpeedHalf"] = (v >> 1) * 2
    values["AccSetSpeedOdd"] = v & 1
  if lead is not None:
    values["AccLeadDetected"] = lead
  if gap_bars is not None and 1 <= gap_bars <= 3:
    values["AccGapBars"] = gap_bars
  # the packer fills CHECKSUM, the torque PDU has its own
  values["CHECKSUM_TORQUE"] = mg4_crc(packer.pack(0x16E, values), 40)
  return packer.make_can_msg("ACC_CMD", bus, values)


def create_mg4_acc_cmd(packer, acc_cmd, accel, decel_regime, set_speed_kph, lead, gap_bars, v_ego, bus):
  P = CarControllerParams
  if decel_regime:
    values = {
      "AccCmdMode": ACC_MODE_REGULATING,
      "AccDecelReq": 1,
      "AccTorqueAct": 0,
      "AccTorqueReq": P.MG4_TORQUE_FLOOR,
      "AccAccelReq": accel,
    }
  else:
    accel = max(P.MG4_COAST_ACCEL_MIN, accel)
    values = {
      "AccCmdMode": ACC_MODE_REGULATING,
      "AccDecelReq": 0,
      "AccTorqueAct": 1,
      "AccTorqueReq": mg4_torque_byte(accel, v_ego),
      "AccAccelReq": accel,
    }
  return _acc_cmd(packer, acc_cmd, values, set_speed_kph, lead, gap_bars, bus)


def create_mg4_acc_hold(packer, acc_cmd, set_speed_kph, gap_bars, bus):
  values = {
    "AccCmdMode": ACC_MODE_HOLD,
    "AccDecelReq": 1,
    "AccTorqueAct": 0,
    "AccTorqueReq": CarControllerParams.MG4_TORQUE_FLOOR,
    "AccAccelReq": ACC_HOLD_ACCEL,
  }
  return _acc_cmd(packer, acc_cmd, values, set_speed_kph, None, gap_bars, bus)


def create_mg4_acc_resume(packer, acc_cmd, set_speed_kph, gap_bars, bus):
  values = {
    "AccCmdMode": ACC_MODE_RESUME,
    "AccDecelReq": 0,
    "AccTorqueAct": 1,
    "AccTorqueReq": ACC_RESUME_TORQUE_BYTE,
    "AccAccelReq": 0.,
  }
  return _acc_cmd(packer, acc_cmd, values, set_speed_kph, None, gap_bars, bus)


def create_mg4_acc_passthrough(packer, acc_cmd, set_speed_kph, gap_bars, bus):
  return _acc_cmd(packer, acc_cmd, {}, set_speed_kph, None, gap_bars, bus)


def mg4_acc_radar_braking(acc_cmd) -> bool:
  return bool(acc_cmd["AccDecelReq"]) or acc_cmd["AccAccelReq"] < -0.08  # passive frames carry 180 (-0.07 m/s2)


def create_mg4_cruise_buttons_blank(packer, counter, bus):
  values = {
    "PDU_HEADER": 0x11000008,
    "ROLLING_COUNTER": counter,
    "SET_ME_X8000FFFF": 0x8000FFFF,
  }
  return packer.make_can_msg("CRUISE_BUTTONS", bus, values)


def raw_words(msg):
  # the whole frame as packed signals, so a stock message can be forwarded with some signals changed
  return {k: v for k, v in msg.items() if k.startswith("Raw")}


def create_mg4_acc_state(packer, acc_state, bus):
  values = raw_words(acc_state)
  values["HandsOnWarnLvl"] = 1
  return packer.make_can_msg("ACC_STATE", bus, values)


def calc_checksum(values):
  lka_req_toq = values['LKAReqToqHSC2'] + 0x400
  lka_req_toq_sts = values['LKAReqToqStsHSC2']
  lka_req_toq_v = values['LKAReqToqVHSC2']
  lka_alv_rc = values['LKAAlvRCHSC2']

  combined = ((lka_req_toq << 1) | (lka_req_toq_sts << 12) | lka_req_toq_v) & 0x3FFF
  with_counter = (combined + lka_alv_rc) & 0x3FFF
  checksum = ((~with_counter) + 1) & 0x3FFF

  return checksum


def create_lka_steering(packer, counter, apply_torque, active):
  values = {
    "LKAReqToqHSC2": apply_torque,
    "LKAReqToqVHSC2": 0,
    "LKAAlvRCHSC2": counter,
    "LDWLKAVbnLvlReqHSC2": 0,  # TODO: vibration level?
    "LKASysStsHSC2": 0,
    "LKAReqToqStsHSC2": active,
    "LKASysFltStsHSC2": 0,
    "LKADrvrTkovReqHSC2": 0,
    "LKAReqToqPVHSC2": 0
  }

  values["LKAReqToqPVHSC2"] = calc_checksum(values)
  return packer.make_can_msg("FVCM_HSC2_FrP03", 0, values)


def mg_checksum(address: int, sig, d: bytearray) -> int:
  if address == 0x1EC:
    torque = ((d[0] & 0x7) << 8) | d[1]
    valid = (d[0] >> 3) & 1
    status = (d[6] >> 4) & 0x7
    counter = d[0] >> 4
    return -((torque << 1) + (status << 12) + valid + counter) & 0x7FFF

  crc = 0xFF
  for byte in d[:-1]:
    crc = CRC8J1850[crc ^ byte]
  return crc ^ 0xFF


def mg4_crc(dat: bytes | bytearray, start: int) -> int:
  # MG4 contained PDUs are [header 4][CRC][7 data]: CRC8 SAE J1850, init 0, over the 7 bytes after the CRC byte
  crc = 0
  for byte in dat[start + 1:start + 8]:
    crc = CRC8J1850[crc ^ byte]
  return crc


def mg4_checksum(address: int, sig, d: bytearray) -> int:
  return mg4_crc(d, sig.start_bit // 8)
