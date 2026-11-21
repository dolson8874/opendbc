import unittest

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.landrover.interface import CarInterface
from opendbc.car.landrover.values import CAR, CarControllerParams


class TestMasterCompatibility(unittest.TestCase):
  def get_interface(self):
    cp = CarInterface.get_non_essential_params(CAR.LANDROVER_DEFENDER_2023)
    cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.LANDROVER_DEFENDER_2023)
    return CarInterface(cp, cp_sp)

  def test_vehicle_limits_and_distinct_safety_ids(self):
    ci = self.get_interface()
    self.assertEqual(int(structs.CarParams.SafetyModel.byd), 35)
    self.assertEqual(int(structs.CarParams.SafetyModel.landrover), 39)
    self.assertEqual(ci.CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.landrover)
    self.assertAlmostEqual(ci.CP.wheelbase, 3.022, places=6)
    self.assertEqual(ci.CP.steerRatio, 19)
    self.assertEqual(CarControllerParams.ANGLE_LIMITS.STEER_ANGLE_MAX, 360)
    self.assertEqual(CarControllerParams.ANGLE_LIMITS.MAX_ANGLE_RATE, 5)
    self.assertAlmostEqual(CarControllerParams.ANGLE_LIMITS.MAX_LATERAL_ACCEL, 3.5886)
    self.assertAlmostEqual(CarControllerParams.ANGLE_LIMITS.MAX_LATERAL_JERK, 3.5886)

  def test_packed_brake_and_blindspot_signals_survive_schema_migration(self):
    ci = self.get_interface()
    ci.update([])  # Register the signals used by the lazy CAN parser.
    packer = CANPacker("landrover_defender_2023")
    messages = [
      packer.make_can_msg("Info02", 0, {"BrakePedalPos": 37}),
      packer.make_can_msg("StopAndGo", 0, {"BrakeDriver": 1}),
      packer.make_can_msg("BlindSpot", 0, {"LeftBS": 4, "RightBS": 0}),
    ]
    state, _ = ci.update([(1_000_000_000, messages)])
    self.assertTrue(state.brakePressed)
    self.assertTrue(state.leftBlindspot)
    self.assertFalse(state.rightBlindspot)
    with structs.CarState.from_bytes(state.to_bytes()) as restored:
      self.assertEqual(restored.deprecated.brake, 37)

  def test_defender_driver_torque_polarity(self):
    ci = self.get_interface()
    ci.update([])
    timestamp = 1_000_000_000
    for magnitude, flags, expected in ((70, 0x00, 70), (70, 0x40, -70), (128, 0x80, 128),
                                       (128, 0xC0, -128), (40, 0x40, -40), (0, 0x40, 0)):
      with self.subTest(magnitude=magnitude, flags=flags):
        for counter in range(10):
          timestamp += 20_000_000
          frame = (0x2E, bytes((0, flags | counter, 0, magnitude)), 0)
          state, _ = ci.update([(timestamp, [frame])])
        self.assertEqual(state.steeringTorque, expected)
        self.assertEqual(state.steeringPressed, magnitude > 40)

  def test_yaw_rate_estimate_from_defender_can(self):
    packer = CANPacker("landrover_defender_2023")
    for speed_kph, angle_deg, gear, yaw_rad_s in (
      (72, 8, 3, 0.03980),
      (72, -8, 3, -0.03980),
      (18, 8, 3, 0.01199),
      (18, 8, 1, -0.01199),
      (0, 45, 3, 0.0),
    ):
      with self.subTest(speed=speed_kph, angle=angle_deg, gear=gear):
        ci = self.get_interface()
        ci.update([])  # Register the signals used by the lazy CAN parser.
        wheel_speeds = {"SpeedLeft": speed_kph, "SpeedRight": speed_kph}
        messages = [
          packer.make_can_msg("WheelSpeedFront", 0, wheel_speeds),
          packer.make_can_msg("WheelSpeedRear", 0, wheel_speeds),
          packer.make_can_msg("SWM_Angle", 0, {"SteerAngle": angle_deg}),
          packer.make_can_msg("GearPRND", 0, {"PRND": gear}),
        ]
        state, _ = ci.update([(1_000_000_000, messages)])
        self.assertAlmostEqual(state.yawRate, yaw_rad_s, places=5)
