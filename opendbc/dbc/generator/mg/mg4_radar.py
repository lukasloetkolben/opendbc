#!/usr/bin/env python3

# MG4 radar object list on the radar's private bus, one object per frame
RADAR_START_ADDR = 0xB0
RADAR_MSG_COUNT = 20


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

BU_: XXX
    """)

  for a in range(RADAR_START_ADDR, RADAR_START_ADDR + 2 * RADAR_MSG_COUNT, 2):
    parts.append(f"""
BO_ {a} RADAR_TRACK_{a:x}: 64 RADAR
 SG_ CHECKSUM : 7|16@0+ (1,0) [0|65535] "" XXX
 SG_ SEQUENCE : 16|8@1+ (1,0) [0|255] "" XXX
 SG_ COUNTER : 24|4@1+ (1,0) [0|15] "" XXX
 SG_ TRACK_ID : 32|8@1+ (1,0) [0|255] "" XXX
 SG_ LONG_DIST : 48|12@1+ (0.05,0) [0|204.75] "m" XXX
 SG_ MEASURED : 60|1@1+ (1,0) [0|1] "" XXX
 SG_ LAT_DIST : 64|14@1+ (0.01,-80) [-80|83.83] "m" XXX
 SG_ REL_SPEED : 80|12@1+ (0.1,-358.4) [-358.4|51.1] "m/s" XXX
 SG_ REL_SPEED_INVALID : 92|1@1+ (1,0) [0|1] "" XXX
    """)

  for a in range(RADAR_START_ADDR, RADAR_START_ADDR + 2 * RADAR_MSG_COUNT, 2):
    parts.append(f"""
CM_ SG_ {a} TRACK_ID "radar track id, 0 = empty slot";
CM_ SG_ {a} LONG_DIST "measured from ~4 m behind the front bumper";
CM_ SG_ {a} LAT_DIST "left positive";
CM_ SG_ {a} REL_SPEED_INVALID "set mostly at ego standstill; REL_SPEED then holds a default";""")

  return {"mg4_radar.dbc": "".join(parts)}
