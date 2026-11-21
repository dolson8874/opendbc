import math
from opendbc.can import CANDefine
from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.landrover import cruise_hud
from opendbc.car.landrover.values import DBC, CanBus, CarControllerParams, LandroverFlags
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from opendbc.sunnypilot.car.landrover.mads import MadsCarState

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase, MadsCarState):
  def __init__(self, CP, CP_SP):
    CarStateBase.__init__(self, CP, CP_SP)
    MadsCarState.__init__(self, CP, CP_SP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])

    if CP.flags & LandroverFlags.FLEXRAY_HARNESS:
      self.shifter_values = can_define.dv["GearPRND"]["PRND"]
      self.VM = VehicleModel(CP)
    else:
      self.shifter_values = can_define.dv["GEAR_PRND"]["GEAR_SHIFT"]

    self.is_metric = True
    self.params = CarControllerParams(CP)
    self.wheelbase = CP.wheelbase

    self.lc_button = 0
    if CP.flags & LandroverFlags.CRUISE_HUD_BENCH:
      self.cruise_hud = cruise_hud.CruiseHud(DBC[CP.carFingerprint][Bus.pt])
    self.block_hud_enable = True
    self.manual_lkas_armed = False
    self.manual_lkas_on = False
    self.lkas_button_received_ns: int | None = None

  def check_lkas_input(self, now_ns):
    if (not self.cruise_hud.transport_valid or
        (self.lkas_button_received_ns is not None and now_ns - self.lkas_button_received_ns >= cruise_hud.LKAS_BUTTON_TIMEOUT_NS)):
      self.manual_lkas_armed = False
      self.manual_lkas_on = False
      self.lkas_button_received_ns = None

  def update_cruise_hud(self, can_packets, now_ns):
    manual_lkas = bool(self.CP.alternativeExperience & ALTERNATIVE_EXPERIENCE.ENABLE_MADS)
    previous = self.cruise_hud.active
    self.cruise_hud.expire(now_ns)
    if manual_lkas:
      self.check_lkas_input(now_ns)
    continuous = self.cruise_hud.active is not None
    for received_ns, frames in can_packets:
      if not 0 <= now_ns - received_ns < cruise_hud.TIMEOUT_NS:
        continue
      for address, data, bus in frames:
        if bus == cruise_hud.BUS and address == cruise_hud.ADDRESS and len(data) == 8:
          self.cruise_hud.receive(data, received_ns)
          continuous &= self.cruise_hud.active is not None
          if manual_lkas:
            self.check_lkas_input(received_ns)
        elif manual_lkas and bus == 0 and address == 0x24 and len(data) == 8 and self.cruise_hud.transport_valid:
          self.lkas_button_received_ns = received_ns
          button_on = bool(data[7] & 0x20)
          if not button_on:
            self.manual_lkas_armed = True
          self.manual_lkas_on = button_on and self.manual_lkas_armed
    self.block_hud_enable = previous is not False or not continuous

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    if self.CP.flags & LandroverFlags.FLEXRAY_HARNESS:
      return self.update_can_defender(can_parsers)

    cp = can_parsers[Bus.pt]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    self.is_metric = True

    ret.seatbeltUnlatched = (cp.vl["SEAT_BELT"]["SEAT_BELT_DRIVER"]  == 0)
    ret.seatbeltUnlatched = False
    ret.doorOpen = False

    self.parse_wheel_speeds(ret,
      cp.vl["SPEED_01"]["SPEED01"],
      cp.vl["SPEED_02"]["SPEED02"],
      cp.vl["SPEED_01"]["SPEED01"],
      cp.vl["SPEED_02"]["SPEED02"],
    )

    ret.standstill = ret.vEgoRaw < 0.01

    ret.steeringAngleDeg = cp.vl["EPS_01"]["STEER_ANGLE01"]
    ret.steeringRateDeg = cp.vl["EPS_01"]["STEER_SPEED01"]

    steer_rad = math.radians(ret.steeringAngleDeg)
    yawrate_rad_s = math.tan(steer_rad) * ret.vEgo / self.wheelbase

    ret.yawRate = math.degrees(yawrate_rad_s)

    # TODO torq TorqEPS Pressed
    ret.steeringTorque = cp.vl["EPS_00"]["STEER_RATE00"]
    ret.steeringTorqueEps = cp.vl["EPS_02"]["STEER_TORQUE_DRIVER02"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.params.STEER_THRESHOLD, 5)

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["TURN_SIGNAL"]['LEFT_BLINK'],cp.vl["TURN_SIGNAL"]['RIGHT_BLINK'])

    gear = cp.vl["GEAR_PRND"]["GEAR_SHIFT"]
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    ret.deprecated.brake = 0
    ret.brakePressed = (cp.vl["CRUISE_CONTROL"]['DRIVER_BRAKE'] == 1)

    gas = (cp.vl["ACCELATOR_DRIVER"]["ACCELATOR_DRIVER"])
    ret.gasPressed = gas >= 1

    if self.CP.deprecated.enableBsm:
      ret.leftBlindspot  = cp.vl["LEFT_ALERT"]["LEFT_ALERT_1"] != 0
      ret.rightBlindspot = cp.vl["RIGHT_ALERT"]["RIGHT_ALERT_1"] != 0
    else:
      ret.leftBlindspot = False
      ret.rightBlindspot = False

    ret.stockAeb = False

    ret.blockPcmEnable = False

    ret.cruiseState.available = cp.vl["CRUISE_CONTROL"]["CRUISE_ON"] == 1
    ret.cruiseState.enabled = cp.vl["CRUISE_CONTROL"]["CRUISE_ON"] == 1
    ret.cruiseState.speed = ret.vEgoRaw
    ret.cruiseState.standstill = False

    return ret, ret_sp

  def update_can_defender(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    self.is_metric = True

    ret.seatbeltUnlatched = (cp.vl["SeatBelt"]["SeatBelt_Driver"]  == 0)
    ret.doorOpen = not any([cp.vl["DoorStatus"]["FrontLeftDoor"],
         cp.vl["DoorStatus"]["FrontRightDoor"],
         cp.vl["DoorStatus"]["RearLeftDoor"],
         cp.vl["DoorStatus"]["RearRightDoor"]])

    self.parse_wheel_speeds(ret,
      cp.vl["WheelSpeedFront"]["SpeedLeft"],
      cp.vl["WheelSpeedFront"]["SpeedRight"],
      cp.vl["WheelSpeedRear"]["SpeedLeft"],
      cp.vl["WheelSpeedRear"]["SpeedRight"],
    )

    ret.standstill = ret.vEgoRaw < 0.01

    ret.steeringAngleDeg = cp.vl["SWM_Angle"]["SteerAngle"]
    ret.steeringRateDeg = cp.vl["SWM_Angle"]["SteerRate"]  # TODO

    # Defender WIP polarity hypothesis: CAN0 0x2E byte 1 bit 6 indicates rightward effort.
    swm_torque = cp.vl["SWM_Torque"]
    magnitude = swm_torque["TorqueDriver"]
    ret.steeringTorque = -magnitude if swm_torque["NEW_SIGNAL_2"] else magnitude
    ret.steeringTorqueEps = cp.vl["PSCM_Out"]["AngleTorque"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.params.STEER_THRESHOLD, 5)

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["HandleSignal"]['TurnLeft'],cp.vl["HandleSignal"]['TurnRight'])

    gear = cp.vl["GearPRND"]["PRND"]
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    # Estimate yaw in rad/s using the nominal vehicle model. Positive is left in forward drive.
    yaw_speed = 0.0 if ret.standstill else max(ret.vEgo, 0.0)
    if ret.gearShifter == structs.CarState.GearShifter.reverse:
      yaw_speed = -yaw_speed
    ret.yawRate = self.VM.yaw_rate(math.radians(ret.steeringAngleDeg), yaw_speed, roll=0.0)

    ret.deprecated.brake = cp.vl["Info02"]["BrakePedalPos"]
    ret.brakePressed = cp.vl["StopAndGo"]["BrakeDriver"] == 1
    ret.gasPressed = cp.vl["GasPedal_ON"]["GasPedalDriver"] == 1

    if self.CP.deprecated.enableBsm:
      ret.leftBlindspot = cp.vl["BlindSpot"]["LeftBS"] == 4
      ret.rightBlindspot = cp.vl["BlindSpot"]["RightBS"] == 4

    ret.stockAeb = False

    if self.CP.flags & LandroverFlags.CRUISE_HUD_BENCH:
      ret.cruiseState.enabled = self.cruise_hud.active is True
      ret.cruiseState.speed = self.cruise_hud.set_speed_kph * CV.KPH_TO_MS
      ret.cruiseState.speedCluster = ret.cruiseState.speed
      ret_sp.stockCruiseGap = self.cruise_hud.gap_level
      ret.blockPcmEnable = self.block_hud_enable
    else:
      ret.cruiseState.enabled = cp.vl["CruiseInfo"]["CruiseOn"] == 1
      ret.cruiseState.speed = ret.vEgoRaw
    ret.cruiseState.nonAdaptive = False
    ret.cruiseState.standstill = False

    #prev_lc_button = self.lc_button
    self.lc_button = bool(cp.vl["LKAS_BTN"]["LKAS_Btn_on"])
    if self.CP.flags & LandroverFlags.CRUISE_HUD_BENCH and self.CP.alternativeExperience & ALTERNATIVE_EXPERIENCE.ENABLE_MADS:
      self.lc_button = self.manual_lkas_on
    ret.cruiseState.available = self.lc_button

    MadsCarState.update_mads(self, ret, can_parsers)

    return ret, ret_sp

  def get_can_parser_defender(self, CP):
    dbc_name = DBC[CP.carFingerprint][Bus.pt]
    parsers = {Bus.pt: CANParser(dbc_name, [], CanBus.UNDERBODY)}
    if CP.flags & LandroverFlags.CRUISE_HUD_BENCH:
      parsers[Bus.adas] = CANParser(dbc_name, [(cruise_hud.ADDRESS, cruise_hud.FREQUENCY_HZ)], cruise_hud.BUS)
    return parsers

  def get_can_parsers(self, CP, CP_SP):
    if CP.flags & LandroverFlags.FLEXRAY_HARNESS:
      return self.get_can_parser_defender(CP)

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus.UNDERBODY),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus.CAM)
    }
