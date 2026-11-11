from opendbc.car.structs import CarParams
from opendbc.car.mg.values import CAR

Ecu = CarParams.Ecu

# TODO: replace with FW versions once the MG4 answers a FW query
FINGERPRINTS = {
  CAR.MG_4_EV: [{
    70: 48, 165: 12, 171: 48, 192: 24, 318: 64, 319: 48, 404: 12, 451: 24, 481: 12, 497: 48, 555: 12, 812: 48, 814: 48,
    924: 24, 1008: 12, 1019: 24, 1142: 24, 1216: 8, 1352: 48, 1358: 12,
  }],
}

FW_VERSIONS = {
  CAR.MG_5_EV: {
    (Ecu.eps, 0x721, None): [
      b'\x10gs\x16\x01',
    ],
    (Ecu.fwdCamera, 0x733, None): [
      b'\x10y\x00 \x01',
    ],
    (Ecu.fwdRadar, 0x734, None): [
      b'\x10y\x000\x01',
    ],
  },
}
