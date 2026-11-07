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
