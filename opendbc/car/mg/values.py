from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
from opendbc.car.lateral import AngleSteeringLimits


class MgFlags(IntFlag):
  CANFD = 1


@dataclass
class MgCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.vw_a]))


@dataclass
class MgPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'mg'})


class CAR(Platforms):
  MG_4_EV = MgPlatformConfig(
    [MgCarDocs("MG4 EV 2022-25")],
    CarSpecs(wheelbase=2.705, steerRatio=15.3),
    {Bus.pt: 'mg4', Bus.radar: 'mg4_radar_generated'},
    flags=MgFlags.CANFD,
  )
  MG_5_EV = MgPlatformConfig([MgCarDocs("MG 5 EV 2021")], CarSpecs(wheelbase=2.66, steerRatio=15.8))


# TODO: the MG4 does not answer this query, it is fingerprinted on CAN
FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"\x10[\x00-\xff]{3}\x01",
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_ECU_HARDWARE_NUMBER_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_ECU_HARDWARE_NUMBER_RESPONSE],
      bus=1,
    ),
  ],
)


class CarControllerParams:
  STEER_STEP = 2  # FVCM_HSC2_FrP03 message frequency 50Hz
  STEER_MAX = 300
  STEER_DELTA_UP = 6       # Max torque reached in 1.00s (STEER_MAX / (50Hz * 1.00))
  STEER_DELTA_DOWN = 10    # Min torque reached in 0.60s (STEER_MAX / (50Hz * 0.60))
  STEER_DRIVER_ALLOWANCE = 100  # allowed driver torque before start limiting
  STEER_DRIVER_MULTIPLIER = 2  # weight driver torque
  STEER_DRIVER_FACTOR = 100

  # MG4: angle steering, rate limits in deg per frame
  MG4_ANGLE_LIMITS = AngleSteeringLimits(
    90.0,  # STEER_ANGLE_MAX (deg at the wheel)
    ([0., 5., 25.], [2.5, 1.5, 0.4]),   # ANGLE_RATE_LIMIT_UP
    ([0., 5., 25.], [5.0, 2.0, 0.8]),   # ANGLE_RATE_LIMIT_DOWN
  )
  MG4_DRIVER_TORQUE_THRESHOLD = 10  # raw counts

  MG4_STEER_AUTHORITY = 200  # EPS torque limit per side in LKA_CMD

  MG4_ACCEL_MAX = 1.0    # m/s2
  MG4_ACCEL_MIN = -3.48  # m/s2, ISO 15622 limit -3.5 after AccAccelReq quantization
  MG4_GAS_OVERRIDE_PCT = 5.0  # %, light pressure stays in the one-pedal regen zone

  # ACC_CMD torque regime: AccTorqueReq with a speed-dependent zero, the floor is no drive torque
  MG4_TORQUE_PER_COUNT = 0.028         # m/s2
  MG4_ACCEL_HYST_GAP = 0.028           # m/s2, one AccTorqueReq count
  MG4_TORQUE_ZERO_BP = [35., 60., 82., 110.]    # km/h
  MG4_TORQUE_ZERO_V = [164., 166., 168., 171.]  # AccTorqueReq giving ~0 m/s2 on the flat
  MG4_TORQUE_FLOOR = 160
  MG4_TORQUE_MAX = 215
  MG4_COAST_ACCEL_MIN = -0.3           # m/s2, lowest request the torque regime can do (floor 160)

  # ACC_CMD decel regime: AccAccelReq sets the braking
  MG4_BRAKE_ENTER = -0.4   # m/s2
  MG4_BRAKE_EXIT = -0.2    # m/s2

  # stop-and-go: hold (mode 10) at standstill, then a resume pulse (mode 5)
  MG4_HOLD_SPEED = 0.25 / 3.6        # m/s
  MG4_RESUME_FRAMES = 25             # ACC_CMD frames (0.5 s)

  def __init__(self, CP):
    pass


DBC = CAR.create_dbc_map()
