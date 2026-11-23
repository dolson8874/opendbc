from opendbc.car.landrover.values import CanBus
from opendbc.car.can_definitions import CanData


# crc8 poly=0x1d, xor=0xcc , 32bit
def defender_crc(data):
   crc = 0
   poly = 0x1d

   for byte in data:
      crc = crc ^ byte
      for _i in range(8):
          if crc & 0x80:
              crc = (crc << 1) ^ poly
          else:
              crc = (crc << 1)
      crc &= 0xFF

   return crc ^ 0xcc


# RR 2017
# 15 all green
# 1d left green, right white
# 35 left white, right green,
def create_lkas_hud(packer, left_line, right_line):
  values = {
    # "GREEN2WHITE_RIGHT": 2 if right_lane_depart else 1 if right_line else 3,
    # "GREEN2WHITE_LEFT": 2 if left_lane_depart else 1 if left_line else 3,
    "GREEN2WHITE_RIGHT": right_line,
    "GREEN2WHITE_LEFT": left_line,
    "NEW_41": 0x41,
    "NEW_01": 1,
    "NEW_0d": 0xd,
    "NEW_1_1": 1,
    "NEW_e7": 0xe7,
    "NEW_2": 2,
    "NEW_ed": 0xed,
    "NEW_00": 0
  }

  return packer.make_can_msg("LKAS_STATUS", 0, values)


# NEOKII RR2017 LKAS_RUN checksum, derived from the counter and torque word.
RR_LKAS_PARITY_MASKS = (0x0D23, 0x1447, 0x2FAD, 0x5878, 0x3FD2, 0x75A4, 0x6748, 0x4091)


def create_rr_lkas_command(frame, torque):
  encoded = 1024 - torque
  high = 0x80 | ((frame % 16) << 3) | (encoded >> 8)
  low = encoded & 0xFF
  word = (high << 8) | low
  checksum = sum(((word & mask).bit_count() & 1) << bit for bit, mask in enumerate(RR_LKAS_PARITY_MASKS))
  payload = bytes((checksum, 0xFF, 0xFF, high, low, 0x70, 0, 0))
  return CanData(0x28F, payload, CanBus.UNDERBODY)


def create_lkas_command_defender(packer, enable, latActive, apply_angle, cnt):

  values = {
    "Lkas_checksum": 0,
    "counter": cnt,
    "ReqAngleTorque": apply_angle,  # todo check
    "EnAngle": latActive,
    "Engaged": enable,
  }

  dat = packer.make_can_msg("LKAS_OP_TO_FLEXRAY", CanBus.CAN2FLEXRAY, values)[1]
  values["Lkas_checksum"] = defender_crc(dat[1:5])

  return packer.make_can_msg("LKAS_OP_TO_FLEXRAY", CanBus.CAN2FLEXRAY, values)


def create_hud_command_defender(packer, enable, latActive, cnt, left_lane, right_lane):

  values = {
    "Lkas_checksum": 0,
    "counter": cnt,
    "EnAngle": latActive,
    "lane_left": left_lane,
    "lane_right": right_lane,
    "Engaged": enable,
  }

  dat = packer.make_can_msg("HUD_OP_TO_FLEXRAY", CanBus.CAN2FLEXRAY, values)[1]
  values["Lkas_checksum"] = defender_crc(dat[1:5])

  return packer.make_can_msg("HUD_OP_TO_FLEXRAY", CanBus.CAN2FLEXRAY, values)
