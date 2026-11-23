import pytest

from opendbc.car import structs
from opendbc.car.landrover.interface import CarInterface
from opendbc.car.landrover.values import CAR


@pytest.mark.parametrize(("direction_bit", "expected"), [(0, 120.0), (1, -120.0)])
def test_rr2017_steering_rate_preserves_magnitude_and_direction(direction_bit, expected):
  cp = CarInterface.get_non_essential_params(CAR.RANGEROVER_VOGUE_2017)
  ci = CarInterface(cp, structs.CarParamsSP())
  ci.update([(1_000_000_000, [])])
  eps_angle = bytes((0, 0, 0, 0, 0x1E, 0x78, 30, 1 | (direction_bit << 1)))
  cs, _ = ci.update([(1_010_000_000, [(0xF2, eps_angle, 0)])])
  assert cs.steeringAngleDeg == 0
  assert cs.steeringRateDeg == expected
