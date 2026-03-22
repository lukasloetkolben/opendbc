#!/usr/bin/env python3

from opendbc.dbc.generator.tesla._radar_common import get_radar_point_definition, get_val_definition


def generate():
  parts = []
  parts.append("""
VERSION ""

NS_ :
	NS_DESC_
	CM_
	BA_DEF_
	BA_
	VAL_
	CAT_DEF_
	CAT_
	FILTER
	BA_DEF_DEF_
	EV_DATA_
	ENVVAR_DATA_
	SGTYPE_
	SGTYPE_VAL_
	BA_DEF_SGTYPE_
	BA_SGTYPE_
	SIG_TYPE_REF_
	VAL_TABLE_
	SIG_GROUP_
	SIG_VALTYPE_
	SIGTYPE_VALTYPE_
	BO_TX_BU_
	BA_DEF_REL_
	BA_REL_
	BA_DEF_DEF_REL_
	BU_SG_REL_
	BU_EV_REL_
	BU_BO_REL_
	SG_MUL_VAL_

BS_:

BU_:  Autopilot Radar Diag

BO_ 80 SpeedInformation: 8 XXX
   SG_ Speed : 0|12@1+ (0.02,-11.18) [0|255] "m/s" XXX
   SG_ Gear : 12|3@1+ (1,0) [0|7] "" XXX
   SG_ Active : 18|1@1+ (1,0) [0|1] "" XXX
   SG_ wheelDirectionFrL : 20|2@1+ (1,0) [0|3] "" XXX
   SG_ vehicleStopping : 22|2@1+ (1,0) [0|3] "" XXX
   SG_ wheelDirectionFrR : 24|2@1+ (1,0) [0|3] "" XXX
   SG_ wheelDirectionReL : 26|2@1+ (1,0) [0|3] "" XXX
   SG_ wheelDirectionReR : 28|2@1+ (1,0) [0|3] "" XXX
   SG_ Counter : 52|4@1+ (1,0) [0|15] "" XXX
   SG_ Checksum : 56|8@1+ (1,0) [0|255] "" XXX

BO_ 81 YawRateInformation: 8 XXX
   SG_ Acceleration : 0|16@1+ (0.001,-32.766) [-32|32] "m/s^2" XXX
   SG_ Unknown1 : 16|8@1+ (1,0) [0|255] "" XXX
   SG_ YawRate : 24|20@1+ (0.0002,-102.4254) [-102.43|107.08] "deg/s" XXX
   SG_ SETME_15 : 44|4@1+ (1,0) [0|15] "" XXX
   SG_ SETME_3 : 48|4@1+ (1,0) [0|15] "" XXX
   SG_ Counter : 52|4@1+ (1,0) [0|15] "" XXX
   SG_ Checksum : 56|8@1+ (1,0) [0|255] "" XXX

BO_ 82 SpeedInformation2: 8 XXX
   SG_ wheelSpeedFrL : 0|13@1+ (0.04,0) [0|327] "kph" XXX
   SG_ wheelSpeedFrR : 13|13@1+ (0.04,0) [0|327] "kph" XXX
   SG_ wheelSpeedReL : 26|13@1+ (0.04,0) [0|327] "kph" XXX
   SG_ wheelSpeedReR : 39|13@1+ (0.04,0) [0|327] "kph" XXX
   SG_ Counter : 52|4@1+ (1,0) [0|15] "" XXX
   SG_ Checksum : 56|8@1+ (1,0) [0|255] "" XXX

BO_ 83 LateralInformation: 8 XXX
   SG_ steeringWheelAngle : 0|14@1+ (0.1,-819.2) [0|65535] "deg" XXX
   SG_ steeringAngleSpeed : 16|14@1+ (0.5,-4096) [0|65535] "deg/s" XXX
   SG_ SETME_3 : 30|22@1+ (1,0) [0|4194303] "" XXX
   SG_ Counter : 52|4@1+ (1,0) [0|15] "" XXX
   SG_ Checksum : 56|8@1+ (1,0) [0|255] "" XXX

BO_ 1025 RadarStatus: 8 Radar
   SG_ carparkDetected : 29|1@1+ (1,0) [0|1] "" Autopilot
   SG_ decreaseBlockage : 25|1@1+ (1,0) [0|1] "" Autopilot
   SG_ horizontMisalignment : 8|12@1+ (0.00012207,-0.25) [-0.25|0.249878] "rad" Autopilot
   SG_ increaseBlockage : 24|1@1+ (1,0) [0|1] "" Autopilot
   SG_ lowPowerMode : 20|2@1+ (1,0) [0|3] "" Autopilot
   SG_ powerOnSelfTest : 22|1@1+ (1,0) [0|1] "" Autopilot
   SG_ sensorBlocked : 26|1@1+ (1,0) [0|1] "" Autopilot
   SG_ sensorInfoConsistBit : 30|1@1+ (1,0) [0|1] "" Autopilot
   SG_ sensorReplace : 31|1@1+ (1,0) [0|1] "" Autopilot
   SG_ shortTermUnavailable : 23|1@1+ (1,0) [0|1] "" Autopilot
   SG_ tunnelDetected : 28|1@1+ (1,0) [0|1] "" Autopilot
   SG_ vehDynamicsError : 27|1@1+ (1,0) [0|1] "" Autopilot
   SG_ verticalMisalignment : 0|8@1+ (0.00195313,-0.25) [-0.25|0.248047] "rad" Autopilot

BO_ 1617 Radar_udsResponse: 8 Radar
   SG_ Radar_udsResponseData : 7|64@0+ (1,0) [0|1.84467e+19] "" Diag

BO_ 1601 UDS_radcRequest: 8 Diag
   SG_ UDS_radcRequestData : 7|64@0+ (1,0) [0|1.84467e+19] "" Radar
""")

  POINT_RANGE = range(0x410, 0x45E + 1, 2)
  for i, base_id in enumerate(POINT_RANGE):
    parts.append(get_radar_point_definition(base_id, f"RadarPoint{i}"))

  parts.append("""
VAL_ 80 Gear 4 "D" 3 "N" 2 "R" 1 "P" 0 "INVALID";
VAL_ 80 wheelDirectionFrL 0 "forward" 2 "standstill" 3 "starting";
VAL_ 80 vehicleStopping 0 "MOVING" 1 "STOPPING";
VAL_ 80 wheelDirectionFrR 0 "forward" 2 "standstill" 3 "starting";
VAL_ 80 wheelDirectionReL 0 "forward" 2 "standstill" 3 "starting";
VAL_ 80 wheelDirectionReR 0 "forward" 2 "standstill" 3 "starting";
VAL_ 1025 lowPowerMode 1 "COMMANDED_LOW_POWER" 0 "DEFAULT_LOW_POWER" 2 "NORMAL_POWER" 3 "SNA";""")

  for base_id in list(POINT_RANGE):
    parts.append(get_val_definition(base_id))

  return {"tesla_radar_continental.dbc": "".join(parts)}
