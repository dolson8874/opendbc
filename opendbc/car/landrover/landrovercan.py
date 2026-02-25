from opendbc.car.landrover.values import CanBus
from opendbc.car.landrover.lkas_crc_table import find_steer_torque
from opendbc.car.can_definitions import CanData
import binascii
import codecs


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

def defender_adas_crc8(cnt: int, poly=0x1D, init=0x00, xorout=0x37) -> int:
   crc = init ^ (cnt & 0xFF)
   for _ in range(8):
      crc = ((crc << 1) ^ poly) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
   return crc ^ xorout

MASKS = [0x0D23, 0x1447, 0x2FAD, 0x5878, 0x3FD2, 0x75A4, 0x6748, 0x4091]

# RR2017
def parity16(v: int) -> int:
   return bin(v & 0xFFFF).count("1") & 1

def checksum_28f(byte3: int, byte4: int) -> int:
   x = ((byte3 & 0xFF) << 8) | (byte4 & 0xFF)
   out = 0
   for i, m in enumerate(MASKS):
      out |= (parity16(x & m) << i)
   return out

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

# LKAS_COMMAND 0x28F (655) Lane-keeping signal to turn the wheel.
def create_lkas_command(packer, lkas_run, frame, apply_steer):
  counter = frame % 0x10
  torque, crc = find_steer_torque(counter, apply_steer)

  #values = {
  #  "CHECKSUM": crc,
  #  "ALLFFFF" : 0xffff,
  #  "A1" : 1,
  #  "HIGH_TORQ": 0,
  #  "ALL11" : 3,
  #  "COUNTER" : counter,
  #  "STEER_TORQ": torque,
  #  "LKAS_GREEN" : 1
  #}

  dat = [0xeb, 0xff, 0xff, 0xe4, 0x00, 0x70, 0x00, 0x00]

  dat[0] = crc
  dat[3] = (((counter << 3) | ((torque & 0x700) >> 8)) | 0x80)
  dat[4] = torque & 0xFF

  candat = binascii.hexlify(bytearray(dat))

  #return packer.make_can_msg("LKAS_RUN", CanBus.UNDERBODY, values)
  return CanData(0x28F,  codecs.decode(candat, 'hex'), CanBus.UNDERBODY)


def create_lkas_command_defender(packer, enable, latActive, apply_angle, cnt):
  if not latActive:
    apply_angle = 0

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

def create_adas_mode(packer, cnt):
  dat = [0x00, 0x00, 0x00, 0x23, 0x28, 0x1A, 0x6C, 0x04]
  dat[1] = cnt % 0x10
  dat[5] = 0x1A
  dat[6] =0x6C
  dat[7] = 0x04

  dat[0] = defender_adas_crc8(dat[1])

  candat = binascii.hexlify(bytearray(dat))
  return CanData(0x203,  codecs.decode(candat, 'hex'), CanBus.UNDERBODY)

def create_adas_mode2(packer, cnt):
  dat = [0x00, 0x00, 0x00, 0x23, 0x28, 0x1A, 0x6C, 0x04]
  dat[1] = cnt % 0x10
  dat[5] = 0x1A
  dat[6] =0x6C
  dat[7] = 0x04

  dat[0] = defender_adas_crc8(dat[1])

  candat = binascii.hexlify(bytearray(dat))
  return CanData(0x203,  codecs.decode(candat, 'hex'), CanBus.CAM)

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
