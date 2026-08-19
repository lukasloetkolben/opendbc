from opendbc.car.rivian.values import RivianFlags


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


def create_lka_steering(packer, frame, acm_lka_hba_cmd, apply_torque, enabled, active):
  # forward auto high beam and speed limit status and nothing else
  values = {s: acm_lka_hba_cmd[s] for s in (
    "ACM_hbaSysState",
    "ACM_hbaLamp",
    "ACM_hbaOnOffState",
    "ACM_slifOnOffState",
  )}

  values |= {
    "ACM_lkaHbaCmd_Counter": frame % 15,  # 0-14 counter (15 states), so % 15 is correct despite the 4-bit field
    "ACM_lkaStrToqReq": apply_torque,
    "ACM_lkaActToi": active,

    "ACM_lkaLaneRecogState": 3 if enabled else 0,
    "ACM_lkaSymbolState": 3 if enabled else 0,

    # static values
    "ACM_lkaElkRequest": 0,
    "ACM_ldwlkaOnOffState": 2,  # 2=LKAS+LDW on
    "ACM_elkOnOffState": 1,  # 1=LKAS on
    # TODO: what are these used for?
    "ACM_ldwWarnTypeState": 2,  # always 2
    "ACM_ldwWarnTimingState": 1,  # always 1
    #"ACM_lkaHandsoffDisplayWarning": 1,  # TODO: we can send this when openpilot wants you to pay attention
  }

  data = packer.make_can_msg("ACM_lkaHbaCmd", 0, values)[1]
  values["ACM_lkaHbaCmd_Checksum"] = checksum(data[1:], 0x1D, 0x63)
  return packer.make_can_msg("ACM_lkaHbaCmd", 0, values)


def create_wheel_touch(packer, sccm_wheel_touch, enabled):
  values = {s: sccm_wheel_touch[s] for s in (
    "SCCM_WheelTouch_Counter",
    "SCCM_WheelTouch_HandsOn",
    "SCCM_WheelTouch_Calibration",
    "SCCM_WheelTouch_CapacitiveValue",
    "SCCM_WheelTouch_ResistiveValue",
  )}

  # When only using ACC without lateral, the ACM warns the driver to hold the steering wheel on engagement
  # Tell the ACM that the user is holding the wheel to avoid this warning
  if enabled:
    values["SCCM_WheelTouch_HandsOn"] = 1
    values["SCCM_WheelTouch_CapacitiveValue"] = 100  # only need to send this value, but both are set for consistency

  data = packer.make_can_msg("SCCM_WheelTouch", 2, values)[1]
  values["SCCM_WheelTouch_Checksum"] = checksum(data[1:], 0x1D, 0x97)
  return packer.make_can_msg("SCCM_WheelTouch", 2, values)


def create_longitudinal(packer, frame, accel, enabled, CP):
  alt_adas = CP.flags & RivianFlags.ALT_ADAS_MESSAGES
  msg_name = "ACM_longitudinalRequest_2" if alt_adas else "ACM_longitudinalRequest"
  values = {
    "ACM_longitudinalRequest_Counter": frame % 15,  # 0-14 counter (15 states), so % 15 is correct despite the 4-bit field
    "ACM_AccelerationRequest": accel,
  }

  if alt_adas:
    # TODO: no ACM_PrndRequest/ACM_VehicleHoldRequest equivalents found yet (no ACC standstill data)
    values |= {
      "ACM_longInterfaceStatus": 4 if enabled else 1,
      # constant on stock traffic regardless of ACC state
      "ACM_Unkown1": 0x2B,
      "ACM_Unkown2": 0xEA,
      "ACM_Unkown3": 0x60,
    }
  else:
    values |= {
      "ACM_PrndRequest": 0,
      "ACM_longInterfaceEnable": 1 if enabled else 0,
      "ACM_VehicleHoldRequest": 0,
    }

  data = packer.make_can_msg(msg_name, 0, values)[1]
  values["ACM_longitudinalRequest_Checksum"] = checksum(data[1:], 0x1D, 0x3A if alt_adas else 0x12)
  return packer.make_can_msg(msg_name, 0, values)


def create_adas_status(packer, vdm_adas_status, interface_status, CP):
  alt_adas = CP.flags & RivianFlags.ALT_ADAS_MESSAGES
  msg_name = "VDM_AdasSts_2" if alt_adas else "VDM_AdasSts"
  sigs = [
    "VDM_AdasStatus_Checksum",
    "VDM_AdasStatus_Counter",
    "VDM_AdasInterfaceStatus",
    "VDM_AdasFaultStatus",
    "VDM_AdasAccelLimit",
    "VDM_AdasUnkown1",
  ]
  if alt_adas:
    sigs += ["VDM_AdasLongActive", "VDM_AdasUnkown2", "VDM_AdasUnkown3"]
  else:
    sigs += ["VDM_AdasDecelLimit", "VDM_AdasDriverAccelPriorityStatus", "VDM_AdasDriverModeStatus",
             "VDM_UserAdasRequest", "VDM_AdasVehicleHoldStatus"]
  values = {s: vdm_adas_status[s] for s in sigs}

  # TODO: cancel spoof is untested on ALT_ADAS_MESSAGES firmware
  if interface_status is not None:
    values["VDM_AdasInterfaceStatus"] = interface_status

  data = packer.make_can_msg(msg_name, 2, values)[1]
  values["VDM_AdasStatus_Checksum"] = checksum(data[1:], 0x1D, 0x7E if alt_adas else 0xD1)
  return packer.make_can_msg(msg_name, 2, values)
