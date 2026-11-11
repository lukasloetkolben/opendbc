from opendbc.car.crc import CRC8J1850

# LKA_CMD: angle setpoint and the EPS torque limit to the left (A) and right (B)
def create_mg4_lka_steering(counter, angle_deg, active, authority_left, authority_right, bus):
  dat = bytearray(24)
  dat[0:4] = b"\x02\x09\x07\x08"                      # first block header (const)
  dat[5] = ((0x4 if active else 0x0) << 4) | (counter & 0xF)
  raw_angle = int(round(angle_deg * 16)) + 32600
  raw_angle = max(0, min(0xFFFF, raw_angle))
  dat[6] = (raw_angle >> 8) & 0xFF
  dat[7] = raw_angle & 0xFF
  dat[8:12] = b"\x80\x1f\xff\xff"                     # observed constant
  dat[12:16] = b"\x02\x09\x11\x08"                    # second block header (const)
  a_raw = max(0, min(0x7FF, int(authority_left) + 1024))
  b_raw = max(0, min(0x7FF, int(authority_right) + 1024))
  packed = (a_raw << 13) | (b_raw << 2)
  dat[16:19] = packed.to_bytes(3, "big")
  # bytes 19-23 stay 0
  dat[4] = mg4_crc(dat, 4)
  return 0x089, bytes(dat), bus


# ACC_STATE with the hands-on warning level forced to OK
def create_mg4_acc_state(base: bytes, bus: int):
  dat = bytearray(base[:48])
  dat[29] = (dat[29] & ~0x30) | 0x10
  return 0x1DA, bytes(dat), bus


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
