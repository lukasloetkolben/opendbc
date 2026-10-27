import glob
import os
import random
import unittest
from unittest import mock

from opendbc import DBC_PATH, get_generated_dbcs
from opendbc.can import CANPacker, CANParser
from opendbc.can.dbc import DBC
from opendbc.can.parser import PyCANParser

DBCS = sorted({os.path.basename(f)[:-4] for f in glob.glob(os.path.join(DBC_PATH, "*.dbc"))} | set(get_generated_dbcs()))


def random_values(rng, msg):
  # leave COUNTER out so the packer increments it and computes a valid checksum
  return {name: rng.randint(0, (1 << min(sig.size, 16)) - 1) * sig.factor + sig.offset
          for name, sig in msg.sigs.items() if name != "COUNTER" and not name.endswith("Counter") and sig.calc_checksum is None}


def make_frames(rng, dbc, packer, bus):
  frames = []
  for msg in dbc.msgs.values():
    r = rng.random()
    if r < 0.7:
      frames.append(packer.make_can_msg(msg.name, bus, random_values(rng, msg)))
    elif r < 0.9:
      frames.append((msg.address, bytes(rng.getrandbits(8) for _ in range(msg.size)), bus))
    # else: drop the message this step
  frames.append((0x7ff, b"\x00" * 8, bus + 1))
  return frames


class TestRustParser(unittest.TestCase):
  def _compare(self, dbc_name, seed):
    rng = random.Random(seed)
    dbc = DBC(dbc_name)
    packer = CANPacker(dbc_name)

    logs = {"py": [], "rs": []}
    with mock.patch("opendbc.can.parser.carlog.warning", side_effect=logs["py"].append):
      py = PyCANParser(dbc_name, [], 0)
    with mock.patch("opendbc.can.parser.carlog.warning", side_effect=logs["rs"].append):
      rs = CANParser(dbc_name, [], 0)

    # half the messages up front with a frequency, the rest lazily through vl
    msgs = list(dbc.msgs.values())
    for i, msg in enumerate(msgs):
      if i % 2 == 0:
        freq = rng.choice([10, 50, 100, float("nan")])
        py._add_message(msg.name, freq)
        rs._add_message(msg.name, freq)

    t = 0
    for step in range(60):
      for m in msgs:
        py.vl[m.name]
        rs.vl[m.name]

      # mostly 10ms steps, with gaps that trigger timeouts and log rate limiting
      t += rng.choice([10_000_000] * 8 + [700_000_000, 3_000_000_000])
      frames = make_frames(rng, dbc, packer, 0) if rng.random() > 0.1 else []
      strings = [(t, frames)] if rng.random() < 0.8 else (t, frames)

      with mock.patch("opendbc.can.parser.carlog.warning", side_effect=logs["py"].append):
        py_updated = py.update(strings)
        py_valid, py_timeout = py.can_valid, py.bus_timeout
      rs_updated = rs.update(strings)
      rs_valid, rs_timeout = rs.can_valid, rs.bus_timeout

      ctx = f"{dbc_name} step {step}"
      self.assertEqual(py_updated, rs_updated, ctx)
      self.assertEqual(py_valid, rs_valid, ctx)
      self.assertEqual(py_timeout, rs_timeout, ctx)
      self.assertEqual(py.can_invalid_cnt, rs.can_invalid_cnt, ctx)
      for m in msgs:
        self.assertEqual(py.vl[m.address], rs.vl[m.address], f"{ctx} {m.name}")
        self.assertEqual(dict(py.vl_all[m.address]), dict(rs.vl_all[m.address]), f"{ctx} {m.name}")
        self.assertEqual(py.ts_nanos[m.address], rs.ts_nanos[m.address], f"{ctx} {m.name}")
      self.assertEqual(logs["py"], logs["rs"], ctx)

  def test_all_dbcs(self):
    for dbc_name in DBCS:
      with self.subTest(dbc=dbc_name):
        for seed in range(2):
          self._compare(dbc_name, seed)

  def test_same_dict_objects(self):
    # carstates hold on to these dicts, the core must update them in place
    rs = CANParser("toyota_nodsu_pt_generated", [("STEER_ANGLE_SENSOR", 80)], 0)
    vl, vl_all, ts = rs.vl["STEER_ANGLE_SENSOR"], rs.vl_all["STEER_ANGLE_SENSOR"], rs.ts_nanos["STEER_ANGLE_SENSOR"]
    packer = CANPacker("toyota_nodsu_pt_generated")
    rs.update([(10, [packer.make_can_msg("STEER_ANGLE_SENSOR", 0, {"STEER_ANGLE": 12.0})])])
    self.assertIs(vl, rs.vl[0x25])
    self.assertEqual(vl["STEER_ANGLE"], 12.0)
    self.assertEqual(vl_all["STEER_ANGLE"], [12.0])
    self.assertEqual(ts["STEER_ANGLE"], 10)
    rs.update([(20, [])])
    self.assertEqual(vl_all["STEER_ANGLE"], [])


if __name__ == "__main__":
  unittest.main()
