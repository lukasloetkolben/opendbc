from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.mg.values import DBC

RADAR_START_ADDR = 0xB0
RADAR_MSG_COUNT = 20
RADAR_ADDRS = range(RADAR_START_ADDR, RADAR_START_ADDR + 2 * RADAR_MSG_COUNT, 2)

# LONG_DIST is measured from ~4 m behind the front bumper, TODO: confirm with a tape measurement
RADAR_DIST_OFFSET = 4.0


def get_radar_can_parser(CP):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None
  messages = [(f"RADAR_TRACK_{addr:x}", 14) for addr in RADAR_ADDRS]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.rcp = None if CP.radarUnavailable else get_radar_can_parser(CP)
    self.trigger_msg = RADAR_START_ADDR
    self.updated_messages = set()

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)
    if self.trigger_msg not in self.updated_messages:
      return None
    self.updated_messages.clear()

    ret = structs.RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    seen = set()
    for addr in RADAR_ADDRS:
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]
      track_id = int(msg["TRACK_ID"])
      if track_id == 0 or not msg["MEASURED"] or msg["REL_SPEED_INVALID"]:
        continue

      # objects move between slots, so points follow the radar's track id
      seen.add(track_id)
      if track_id not in self.pts:
        self.pts[track_id] = structs.RadarData.RadarPoint()
        self.pts[track_id].trackId = self.track_id
        self.track_id += 1

      self.pts[track_id].dRel = msg["LONG_DIST"] - RADAR_DIST_OFFSET
      self.pts[track_id].yRel = msg["LAT_DIST"]
      self.pts[track_id].vRel = msg["REL_SPEED"]

    for track_id in set(self.pts) - seen:
      del self.pts[track_id]

    ret.points = list(self.pts.values())
    return ret
