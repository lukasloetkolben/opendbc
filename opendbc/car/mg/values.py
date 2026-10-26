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
    {Bus.pt: 'mg4'},
    flags=MgFlags.CANFD,
  )
  MG_5_EV = MgPlatformConfig([MgCarDocs("MG 5 EV 2021")], CarSpecs(wheelbase=2.66, steerRatio=15.8))


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

  # MG4 (CANFD): angle-based LKA_CMD (0x089) at 100Hz. Limits are conservative for first bring-up;
  # tune once the EPS is confirmed to accept the command. Rate limits are deg per 10ms frame.
  MG4_STEER_STEP = 1
  MG4_ANGLE_LIMITS = AngleSteeringLimits(
    90.0,  # STEER_ANGLE_MAX (deg at the wheel)
    ([0., 5., 25.], [2.5, 1.5, 0.4]),   # ANGLE_RATE_LIMIT_UP
    ([0., 5., 25.], [5.0, 2.0, 0.8]),   # ANGLE_RATE_LIMIT_DOWN
  )

  def __init__(self, CP):
    pass


DBC = CAR.create_dbc_map()
