from opendbc.car.crc import CRC8J1850


def mg4_lka_checksum(dat: bytes) -> int:
  # CRC8, SAE J1850 poly 0x1D
  crc = 0
  for b in dat:
    crc ^= b
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
  return crc


# MG4 LKA_CMD (0x089): 24-byte angle command. Verified against route 73d49d6e14131585/00000026:
#   byte 4  = CRC8(0x1D, init 0) over bytes [5:12]        (5000/5000 frames)
#   byte 5  = state<<4 | counter   (state 4 = active, counter cycles 0..14)
#   bytes 6-7 = steering wheel angle, big-endian, raw = angle_deg*16 + 32600  (neutral 0x7F58)
#   bytes 16-18 = two packed 11-bit torque request fields. DBC values use offset-binary
#                 decoding (raw - 1024); route 73d49d6e14131585/00000024/13 commonly has +150/-150.
def create_mg4_lka_steering(counter, angle_deg, active, torque_a, torque_b, bus):
  dat = bytearray(24)
  dat[0:4] = b"\x02\x09\x07\x08"                      # first block header (const)
  dat[5] = ((0x4 if active else 0x0) << 4) | (counter & 0xF)
  raw_angle = int(round(angle_deg * 16)) + 32600
  raw_angle = max(0, min(0xFFFF, raw_angle))
  dat[6] = (raw_angle >> 8) & 0xFF
  dat[7] = raw_angle & 0xFF
  dat[8:12] = b"\x80\x1f\xff\xff"                     # observed constant
  dat[12:16] = b"\x02\x09\x11\x08"                    # second block header (const)
  torque_a_raw = max(0, min(0x7FF, int(torque_a) + 1024))
  torque_b_raw = max(0, min(0x7FF, int(torque_b) + 1024))
  packed_torque = (torque_a_raw << 13) | (torque_b_raw << 2)
  dat[16:19] = packed_torque.to_bytes(3, "big")
  # bytes 19-23 stay 0
  dat[4] = mg4_lka_checksum(dat[5:12])
  return 0x089, bytes(dat), bus


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
