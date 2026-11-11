import unittest

from opendbc.car.structs import CarParams
from opendbc.car.mg.mgcan import (create_mg4_lka_steering, mg4_crc, create_mg4_acc_cmd, create_mg4_acc_hold,
                                  create_mg4_acc_resume, create_mg4_acc_passthrough, create_mg4_cruise_buttons_blank,
                                  create_mg4_acc_state)
from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import state_buffer

MG_PARAM_CANFD = 1
MG_PARAM_LONGITUDINAL = 2


PACKER = CANPacker("mg4")


def parse(msg, addr, dat, bus=2):
  parser = CANParser("mg4", [(msg, 0)], bus)
  parser.update([(0, [(addr, bytes(dat), bus)])])
  return dict(parser.vl[msg])


def radar_acc_cmd(counter=0, mode=2):
  # a passive ACC_CMD as the radar sends it: four sub-frames [02 34 id 08][crc][7 data], AccAccelReq 180, AccTorqueReq 160
  dat = bytearray(48)
  for i, sub in enumerate((0x07, 0x11, 0x1B, 0x25)):
    dat[12 * i:12 * i + 4] = bytes((0x02, 0x34, sub, 0x08))
  dat[5] = (mode << 4) | counter
  dat[7] = 180
  dat[47] = 160
  dat[4] = mg4_crc(dat, 4)
  dat[40] = mg4_crc(dat, 40)
  return dat


def radar_acc_state(counter=0, acc_sts=0, warn=1):
  dat = bytearray(48)
  dat[0:4] = bytes((0x02, 0x09, 0x00, 0x08))
  dat[5] = counter
  dat[17] = acc_sts << 5
  dat[29] = 0x40 | (warn << 4)
  return dat


class TestMG4Safety(unittest.TestCase):
  PARAM = MG_PARAM_CANFD

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.assertEqual(self.safety.set_safety_hooks(CarParams.SafetyModel.mg, self.PARAM), 0)
    self.safety.init_tests()

  def tearDown(self):
    # the hooks run on two state copies, MG4 state must live in SafetyState so both copies agree
    self.assertFalse(self.safety.safety_redundancy_fault)
    self.assertEqual(state_buffer("safety_a")[:], state_buffer("safety_b")[:])

  def packet(self, addr, bus, data):
    return libsafety_py.make_CANPacket(addr, bus, data)

  def command(self, angle=0, active=True, a=200, b=-200):
    _, data, _ = create_mg4_lka_steering(PACKER, 0, angle, active, a, b, 0)
    return bytearray(data)

  def tx(self, data, bus=0, addr=0x89):
    return self.safety.safety_tx_hook(self.packet(addr, bus, data))

  def rx(self, addr, bus, data):
    crc = 28 if addr in (0x46, 0x13E) else 4  # the PDU that carries the checked signals
    data[crc] = mg4_crc(data, crc)
    return self.safety.safety_rx_hook(self.packet(addr, bus, data))

  def test_engagement_and_brake(self):
    for state in range(8):
      self.setUp()
      data = bytearray(48)
      data[17] = state << 5
      self.assertTrue(self.rx(0x1DA, 2, data))
      self.assertEqual(self.safety.get_controls_allowed(), state in (1, 2, 3, 4))
    self.safety.set_controls_allowed(True)
    data = bytearray(48)
    data[29] = 0x20
    self.assertTrue(self.rx(0x46, 0, data))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_controls_and_allowlist(self):
    self.assertFalse(self.tx(self.command()))
    self.assertTrue(self.tx(self.command(active=False, a=0, b=0)))
    self.safety.set_controls_allowed(True)
    self.assertTrue(self.tx(self.command()))
    self.assertFalse(self.tx(self.command(), bus=2))
    self.assertFalse(self.tx(self.command(), addr=0x1DA))
    self.assertFalse(self.tx(self.command()[:12]))

  def test_forwarding(self):
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x89), -1)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x1DA), -1)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x16E), 0)
    self.assertEqual(self.safety.safety_fwd_hook(0, 0x1E1), 2)
    self.assertEqual(self.safety.safety_fwd_hook(0, 0xAB), 2)

  def test_authority_envelope(self):
    # each side within 0..200 in its own direction; inactive commands request zero
    for a, b, allowed in ((200, -200, True), (150, -150, True), (201, -200, False), (200, -201, False),
                          (-1, -200, False), (200, 1, False), (0, 0, True)):
      self.setUp()
      self.safety.set_controls_allowed(True)
      self.assertEqual(bool(self.tx(self.command(a=a, b=b))), allowed)
    self.assertFalse(self.tx(self.command(active=False)))

  def test_angle_limits(self):
    for angle in (-91, 91):
      self.setUp()
      self.safety.set_controls_allowed(True)
      self.safety.set_desired_angle_last(round(angle * 16))
      self.assertFalse(self.tx(self.command(angle=angle)))
    self.setUp()
    self.safety.set_controls_allowed(True)
    self.assertTrue(self.tx(self.command(angle=1)))
    self.assertFalse(self.tx(self.command(angle=40)))

  def test_malformed_commands(self):
    for index, mask in ((0, 1), (4, 1), (8, 1), (12, 1), (18, 1), (23, 1)):
      self.setUp()
      self.safety.set_controls_allowed(True)
      data = self.command()
      data[index] ^= mask
      self.assertFalse(self.tx(data))

  def test_rx_checksum(self):
    # each message is checked on the PDU that carries its signals: STEER_ANGLE at byte 0, GAS_PEDAL at byte 24
    for addr, crc in ((0xAB, 4), (0x46, 28)):
      data = bytearray(48)
      self.assertTrue(self.rx(addr, 0, data))
      data[crc] ^= 1
      self.assertFalse(self.safety.safety_rx_hook(self.packet(addr, 0, data)))

  def test_measured_angle_and_speed(self):
    data = bytearray(48)
    data[6:8] = (32768 + 16 * 20).to_bytes(2, 'big')
    for _ in range(6):
      self.assertTrue(self.rx(0xAB, 0, data))
    self.assertTrue(self.tx(self.command(angle=20, active=False, a=0, b=0)))
    self.assertFalse(self.tx(self.command(angle=0, active=False, a=0, b=0)))
    speed = bytearray(64)
    speed[30:32] = (5760).to_bytes(2, 'big')  # 90 km/h = 25 m/s
    for _ in range(6):
      self.assertTrue(self.rx(0x13E, 0, speed))
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_angle_last(0)
    self.assertFalse(self.tx(self.command(angle=2)))

  def test_acc_state_forward(self):
    # the radar's ACC_STATE is re-sent on bus 0 with the hands-on warning forced to OK and nothing else changed
    raw = radar_acc_state(counter=3, acc_sts=2, warn=0)
    self.assertFalse(self.tx(raw, addr=0x1DA))   # nothing received yet
    self.assertTrue(self.rx(0x1DA, 2, raw))
    _, data, _ = create_mg4_acc_state(PACKER, parse("ACC_STATE", 0x1DA, raw), 0)
    self.assertTrue(self.tx(bytearray(data), addr=0x1DA))
    self.assertEqual(data[29] & 0x30, 0x10)
    self.assertFalse(self.tx(raw, addr=0x1DA))   # the warning left as is
    data = bytearray(data)
    data[17] ^= 0x20
    self.assertFalse(self.tx(data, addr=0x1DA))

  def test_no_longitudinal_without_param(self):
    base = radar_acc_cmd()
    self.assertTrue(self.rx(0x16E, 2, bytearray(base)))
    self.safety.set_controls_allowed(True)
    self.assertFalse(self.tx(bytearray(base), addr=0x16E))
    self.assertFalse(self.tx(bytearray(create_mg4_cruise_buttons_blank(PACKER, 0, 2)[1]), bus=2, addr=0x1E1))

  def test_dbc_torque_layout(self):
    parser = CANParser('mg4', [('LKA_CMD', 100)], 2)
    parser.update([(0, [(0x89, self.command(), 2)])])
    self.assertEqual(parser.vl['LKA_CMD']['LKAReqToqA'], 200)
    self.assertEqual(parser.vl['LKA_CMD']['LKAReqToqB'], -200)


class TestMG4LongitudinalSafety(TestMG4Safety):
  PARAM = MG_PARAM_CANFD | MG_PARAM_LONGITUDINAL

  def speed(self, kph):
    data = bytearray(64)
    data[30:32] = round(kph * 64).to_bytes(2, 'big')
    for _ in range(6):
      self.assertTrue(self.rx(0x13E, 0, data))

  def acc_cmd(self, accel, decel_regime=False, base=None, v_kph=80):
    acc_cmd = parse("ACC_CMD", 0x16E, base or radar_acc_cmd())
    _, data, _ = create_mg4_acc_cmd(PACKER, acc_cmd, accel, decel_regime, 100, False, 2, v_kph / 3.6, 0)
    return bytearray(data)

  def buttons(self, counter=0, set_=0, up=0, down=0, click=0):
    data = bytearray(create_mg4_cruise_buttons_blank(PACKER, counter, 2)[1])
    data[5] |= set_ << 5
    data[7] |= up << 7
    data[6] |= (down << 2) | (click << 3)
    return data

  def test_engagement_and_brake(self):
    # with openpilot longitudinal the car's ACC state does not engage; the buttons and pedals do
    for state in range(8):
      data = bytearray(48)
      data[17] = state << 5
      self.assertTrue(self.rx(0x1DA, 2, data))
      self.assertFalse(self.safety.get_controls_allowed())
    for press, release, allowed_after in ((dict(set_=1), dict(), True), (dict(set_=1), dict(), False),
                                          (dict(up=1), dict(), True), (dict(down=1), dict(), True), (dict(up=1), dict(), True)):
      self.assertTrue(self.rx(0x1E1, 0, self.buttons(**press)))
      self.assertTrue(self.rx(0x1E1, 0, self.buttons(**release)))
      self.assertEqual(self.safety.get_controls_allowed(), allowed_after)
    self.safety.set_controls_allowed(True)
    self.assertTrue(self.rx(0x1E1, 0, self.buttons(down=1)))
    self.assertTrue(self.rx(0x1E1, 0, self.buttons()))
    self.assertTrue(self.safety.get_controls_allowed())
    data = bytearray(48)
    data[29] = 0x20
    self.assertTrue(self.rx(0x46, 0, data))
    self.assertFalse(self.safety.get_controls_allowed())
    # gas is an override, not a disengage: controls stay allowed but only the radar's own frame may go out
    self.safety.set_controls_allowed(True)
    self.assertTrue(self.rx(0x16E, 2, radar_acc_cmd()))
    self.assertTrue(self.tx(self.acc_cmd(0.5), addr=0x16E))
    data = bytearray(48)
    data[30] = 13   # > 5 % gas
    self.assertTrue(self.rx(0x46, 0, data))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertFalse(self.tx(self.acc_cmd(0.5), addr=0x16E))
    self.assertTrue(self.tx(bytearray(radar_acc_cmd()), addr=0x16E))

  def test_forwarding(self):
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x89), -1)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x1DA), -1)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x16E), -1)
    self.assertEqual(self.safety.safety_fwd_hook(0, 0x1E1), -1)
    self.assertEqual(self.safety.safety_fwd_hook(0, 0xAB), 2)

  def test_no_longitudinal_without_param(self):
    pass

  def test_blank_buttons(self):
    self.assertTrue(self.tx(self.buttons(counter=5), bus=2, addr=0x1E1))
    for pressed in (dict(set_=1), dict(up=1), dict(down=1), dict(click=1)):
      self.assertFalse(self.tx(self.buttons(**pressed), bus=2, addr=0x1E1))
    self.assertFalse(self.tx(self.buttons(), bus=0, addr=0x1E1))
    data = self.buttons()
    data[4] ^= 1
    self.assertFalse(self.tx(data, bus=2, addr=0x1E1))

  def test_acc_cmd_passthrough(self):
    base = radar_acc_cmd(counter=7)
    self.assertFalse(self.tx(bytearray(base), addr=0x16E))   # nothing received yet
    self.assertTrue(self.rx(0x16E, 2, bytearray(base)))
    self.assertTrue(self.tx(bytearray(base), addr=0x16E))    # exact copy, no controls needed
    _, data, _ = create_mg4_acc_passthrough(PACKER, parse("ACC_CMD", 0x16E, base), 120, 3, 0)
    self.assertTrue(self.tx(bytearray(data), addr=0x16E))    # cluster fields may differ
    data = bytearray(base)
    data[7] = 190
    self.assertFalse(self.tx(data, addr=0x16E))

  def test_acc_cmd_regulating(self):
    self.assertTrue(self.rx(0x16E, 2, radar_acc_cmd()))
    self.assertFalse(self.tx(self.acc_cmd(0.0), addr=0x16E))   # not engaged: only the radar's own frame
    self.safety.set_controls_allowed(True)
    for accel in (0.0, 0.5, 1.0, -0.3):
      self.assertTrue(self.tx(self.acc_cmd(accel), addr=0x16E))
    for accel in (-0.5, -1.0, -1.5, -3.48):
      self.assertTrue(self.tx(self.acc_cmd(accel, decel_regime=True), addr=0x16E))
    data = self.acc_cmd(-0.3)
    data[46] &= 0xEF   # neither torque nor decel regime
    data[40] = mg4_crc(data, 40)
    self.assertFalse(self.tx(data, addr=0x16E))
    data = self.acc_cmd(1.0)
    data[7] = 222   # +1.56 m/s2
    data[4] = mg4_crc(data, 4)
    self.assertFalse(self.tx(data, addr=0x16E))
    data = self.acc_cmd(-1.0, decel_regime=True)
    data[7] = 90   # -3.54 m/s2, one count past the ISO limit
    data[4] = mg4_crc(data, 4)
    self.assertFalse(self.tx(data, addr=0x16E))
    data = self.acc_cmd(0.5)
    data[4] ^= 1
    self.assertFalse(self.tx(data, addr=0x16E))
    data = self.acc_cmd(0.5)
    data[5] = (6 << 4) | (data[5] & 0x0F)   # gas-override mode may not be synthesised
    data[4] = mg4_crc(data, 4)
    self.assertFalse(self.tx(data, addr=0x16E))

  def test_hold_and_resume(self):
    self.assertTrue(self.rx(0x16E, 2, radar_acc_cmd()))
    self.safety.set_controls_allowed(True)
    radar = parse("ACC_CMD", 0x16E, radar_acc_cmd())
    _, hold, _ = create_mg4_acc_hold(PACKER, radar, 100, 2, 0)
    _, resume, _ = create_mg4_acc_resume(PACKER, radar, 100, 2, 0)
    self.speed(0)
    self.assertTrue(self.tx(bytearray(hold), addr=0x16E))
    self.assertTrue(self.tx(bytearray(resume), addr=0x16E))
    self.speed(2)
    self.assertFalse(self.tx(bytearray(hold), addr=0x16E))
    self.assertTrue(self.tx(bytearray(resume), addr=0x16E))
    self.speed(5)
    self.assertFalse(self.tx(bytearray(hold), addr=0x16E))
    self.assertFalse(self.tx(bytearray(resume), addr=0x16E))
    self.safety.set_controls_allowed(False)
    self.speed(0)
    self.assertFalse(self.tx(bytearray(hold), addr=0x16E))


if __name__ == "__main__":
  unittest.main()
