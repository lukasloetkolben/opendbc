import unittest

from opendbc.car.structs import CarParams
from opendbc.car.mg.mgcan import create_mg4_lka_steering, mg4_lka_checksum
from opendbc.can.parser import CANParser
from opendbc.safety.tests.libsafety import libsafety_py


class TestMG4Safety(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.assertEqual(self.safety.set_safety_hooks(CarParams.SafetyModel.mg, 1), 0)
    self.safety.init_tests()

  def packet(self, addr, bus, data):
    return libsafety_py.make_CANPacket(addr, bus, data)

  def command(self, angle=0, active=True, a=150, b=-150):
    _, data, _ = create_mg4_lka_steering(0, angle, active, a, b, 0)
    return bytearray(data)

  def tx(self, data, bus=0, addr=0x89):
    return self.safety.safety_tx_hook(self.packet(addr, bus, data))

  def rx(self, addr, bus, data):
    data[4] = mg4_lka_checksum(data[5:12])
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
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x1DA), 0)
    self.assertEqual(self.safety.safety_fwd_hook(0, 0xAB), 2)

  def test_torque_envelope(self):
    for a, b, allowed in ((150, -150, True), (151, -150, False), (150, -151, False),
                           (-1, -150, False), (150, 1, False), (0, 0, True)):
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
    data = bytearray(48)
    self.assertTrue(self.rx(0xAB, 0, data))
    data[4] ^= 1
    self.assertFalse(self.safety.safety_rx_hook(self.packet(0xAB, 0, data)))

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

  def test_dbc_torque_layout(self):
    parser = CANParser('mg4', [('LKA_CMD', 100)], 2)
    parser.update([(0, [(0x89, self.command(), 2)])])
    self.assertEqual(parser.vl['LKA_CMD']['LKAReqToqA'], 150)
    self.assertEqual(parser.vl['LKA_CMD']['LKAReqToqB'], -150)
