def checksum(data, poly, xor_output):
  crc = 0
  for byte in data:
    crc ^= byte
    for _ in range(8):
      if crc & 0x80:
        crc = (crc << 1) ^ poly
      else:
        crc <<= 1
      crc &= 0xFF
  return crc ^ xor_output


def create_acm_status(packer, frame, active):
  values = {
    "ACM_Status_Counter": frame % 15,
    "ACM_FeatureStatus": 3 if active else 0,
    "ACM_FaultStatus": 0,
    "ACM_Unkown2": 0
  }

  data = packer.make_can_msg("ACM_Status", 0, values)[1]
  values["ACM_Status_Checksum"] = checksum(data[1:], 0x1D, 0x5F)
  return packer.make_can_msg("ACM_Status", 0, values)

def create_angle_steering(packer, frame, angle, active):
  values = {
    "ACM_SteeringControl_Counter": frame % 15,
    "ACM_SteeringAngleRequest": angle,
    "ACM_EacEnabled": active,
    "ACM_HapticRequired": 0
  }

  data = packer.make_can_msg("ACM_SteeringControl", 0, values)[1]
  values["ACM_SteeringControl_Checksum"] = checksum(data[1:], 0x1D, 0x41)
  return packer.make_can_msg("ACM_SteeringControl", 0, values)

def create_lka_steering(packer, frame, acm_lka_hba_cmd, active):
  # forward auto high beam and speed limit status and nothing else
  values = {s: acm_lka_hba_cmd[s] for s in (
    "ACM_hbaSysState", # 1
    "ACM_hbaLamp", # 1
    "ACM_hbaOnOffState", # 1
    "ACM_slifOnOffState", #1
  )}

  values |= {
    "ACM_lkaHbaCmd_Counter": frame % 15,
    "ACM_lkaStrToqReq" : 0,
    "ACM_lkaActToi": 0,

    "ACM_lkaLaneRecogState": 3 if active else 0,
    "ACM_lkaSymbolState": 2 if active else 0,

    # static values
    "ACM_ldwLHWarning": 0,
    "ACM_HapticRequest": 0,
    "ACM_lkaToiFlt": 0,
    "ACM_lkaElkRequest": 0,
    "ACM_ldwlkaOnOffState": 2,  # 2=LKAS+LDW on
    "ACM_elkOnOffState": 1,  # 1=LKAS on
    # TODO: what are these used for?
    "ACM_ldwWarnTypeState": 1,
    "ACM_ldwWarnTimingState": 2,  # always 1
    "ACM_lkaHandsoffSoundWarning": 0,
    "ACM_lkaHandsoffDisplayWarning": 0,  # TODO: we can send this when openpilot wants you to pay attention
    "ACM_ldwRHWarning": 0,
  }

  data = packer.make_can_msg("ACM_lkaHbaCmd", 0, values)[1]
  values["ACM_lkaHbaCmd_Checksum"] = checksum(data[1:], 0x1D, 0x63)
  return packer.make_can_msg("ACM_lkaHbaCmd", 0, values)


def create_wheel_touch(packer, sccm_wheel_touch, enabled):
  values = {s: sccm_wheel_touch[s] for s in (
    "SCCM_WheelTouch_Counter",
    "SCCM_WheelTouch_HandsOn",
    "SCCM_WheelTouch_CapacitiveValue",
    "SETME_X52",
  )}

  # When only using ACC without lateral, the ACM warns the driver to hold the steering wheel on engagement
  # Tell the ACM that the user is holding the wheel to avoid this warning
  if enabled:
    values["SCCM_WheelTouch_HandsOn"] = 1
    values["SCCM_WheelTouch_CapacitiveValue"] = 100  # only need to send this value, but both are set for consistency

  data = packer.make_can_msg("SCCM_WheelTouch", 2, values)[1]
  values["SCCM_WheelTouch_Checksum"] = checksum(data[1:], 0x1D, 0x97)
  return packer.make_can_msg("SCCM_WheelTouch", 2, values)


def create_longitudinal(packer, frame, accel, enabled):
  values = {
    "ACM_longitudinalRequest_Counter": frame % 15,
    "ACM_AccelerationRequest": accel if enabled else 0,
    "ACM_PrndRequest": 0,
    "ACM_longInterfaceEnable": 1 if enabled else 0,
    "ACM_VehicleHoldRequest": 0,
  }

  data = packer.make_can_msg("ACM_longitudinalRequest", 0, values)[1]
  values["ACM_longitudinalRequest_Checksum"] = checksum(data[1:], 0x1D, 0x12)
  return packer.make_can_msg("ACM_longitudinalRequest", 0, values)


def create_adas_status(packer, vdm_adas_status, interface_status):
  values = {s: vdm_adas_status[s] for s in (
    "VDM_AdasStatus_Checksum",
    "VDM_AdasStatus_Counter",
    "VDM_AdasDecelLimit",
    "VDM_AdasDriverAccelPriorityStatus",
    "VDM_AdasFaultStatus",
    "VDM_AdasAccelLimit",
    "VDM_AdasDriverModeStatus",
    "VDM_AdasUnkown1",
    "VDM_AdasInterfaceStatus",
    "VDM_AdasVehicleHoldStatus",
    "VDM_UserAdasRequest",
  )}

  if interface_status is not None:
    values["VDM_AdasInterfaceStatus"] = interface_status

  data = packer.make_can_msg("VDM_AdasSts", 2, values)[1]
  values["VDM_AdasStatus_Checksum"] = checksum(data[1:], 0x1D, 0xD1)
  return packer.make_can_msg("VDM_AdasSts", 2, values)
