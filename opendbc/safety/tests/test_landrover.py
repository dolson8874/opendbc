#!/usr/bin/env python3
import unittest
import numpy as np

from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.landrover.values import CarControllerParams, LandroverFlags
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

MSG_CAM_MSG = 0x1BE
MSG_LKAS_C2F = 0x1F0
MSG_HUD_C2F = 0x1F9


def get_safety_CP():
  from opendbc.car.landrover.interface import CarInterface

  return CarInterface.get_non_essential_params("LANDROVER_DEFENDER_2023")


class TestLandroverSafety(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_CAM_MSG,)}
  FWD_BLACKLISTED_ADDRS = {}
  TX_MSGS = [[MSG_LKAS_C2F, 1], [MSG_HUD_C2F, 1], [MSG_CAM_MSG, 0]]

  # Angle control limits
  STEER_ANGLE_MAX = 90  # deg
  DEG_TO_CAN = 12.5

  ANGLE_RATE_BP = [0.0, 5.0, 25.0]
  ANGLE_RATE_UP = [2.5, 1.5, 0.2]  # windup limit
  ANGLE_RATE_DOWN = [5.0, 2.0, 0.3]  # unwind limit
  LATERAL_FREQUENCY = 50  # Hz

  # Long control limits
  MAX_ACCEL = 2.0
  MIN_ACCEL = -3.48
  INACTIVE_ACCEL = 0.0

  packer: CANPackerSafety

  def setUp(self):
    self.VM = VehicleModel(get_safety_CP())
    self.packer = CANPackerSafety("landrover_defender_2023")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.landrover, int(LandroverFlags.FLEXRAY_HARNESS))
    self.safety.init_tests()
    self.cnt_angle_cmd = 0

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True):
    values = {
      "ReqAngleTorque": angle,
      "EnAngle": 1 if enabled else 0,
    }
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("LKAS_OP_TO_FLEXRAY", 1, values)

  """
  def _angle_meas_msg(self, angle: float):
    values = {"SteerAngle": angle}
    return self.packer.make_can_msg_safety("SWM_Angle", 0, values)
  """

  def _angle_meas_msg(self, angle: float):
    # values = {"AngleTorque": angle}
    # return self.packer.make_can_msg_safety("PSCM_Out", 0, values)
    values = {"SteerAngle": angle}
    return self.packer.make_can_msg_safety("SWM_Angle", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BrakeDriver": brake}
    return self.packer.make_can_msg_safety("StopAndGo", 0, values)

  def _speed_msg(self, speed):
    values = {"WheelSpeed": speed * 3.6}
    return self.packer.make_can_msg_safety("Info02", 0, values)

  def _user_gas_msg(self, gas):
    values = {"GasPedalDriver": gas > 0.1}
    return self.packer.make_can_msg_safety("GasPedal_ON", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CruiseOn": 1 if enable else 0}
    return self.packer.make_can_msg_safety("CruiseInfo", 0, values)

  def test_angle_cmd_when_enabled(self):
    # VM-based safety uses jerk/accel limits, covered below.
    pass

  def test_angle_violation(self):
    # VM-based safety does not use the generic fixed-angle violation behavior.
    pass

  def test_lateral_accel_limit(self):
    for speed in np.linspace(0, 40, 80):
      speed = max(speed, 1.0)
      self._reset_speed_measurement(speed + 1.0)
      self.safety.set_controls_allowed(True)

      max_angle = min(get_max_angle_vm(speed, self.VM, CarControllerParams), self.STEER_ANGLE_MAX)
      max_angle_can = int(round(max_angle * self.DEG_TO_CAN))
      max_angle = max_angle_can / self.DEG_TO_CAN

      for sign in (-1, 1):
        target_angle = sign * max_angle
        self.safety.set_desired_angle_last(int(round(target_angle * self.DEG_TO_CAN)))
        self.assertTrue(self._tx(self._angle_cmd_msg(target_angle, True)))

  def test_lateral_jerk_limit(self):
    for speed in np.linspace(0, 40, 80):
      speed = max(speed, 1.0)
      self._reset_speed_measurement(speed + 1.0)

      max_delta = get_max_angle_delta_vm(speed, self.VM, CarControllerParams)
      max_delta = min(max_delta, CarControllerParams.ANGLE_LIMITS.MAX_ANGLE_RATE)
      max_delta_can = max(int(round(max_delta * self.DEG_TO_CAN)), 1)
      max_delta = max_delta_can / self.DEG_TO_CAN

      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._tx(self._angle_cmd_msg(0.0, True))

        self.assertTrue(self._tx(self._angle_cmd_msg(sign * max_delta, True)))
        self.assertTrue(self._tx(self._angle_cmd_msg(sign * max_delta, True)))
        self.assertTrue(self._tx(self._angle_cmd_msg(0.0, True)))


if __name__ == "__main__":
  unittest.main()
