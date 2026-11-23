import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.landrover.interface import CarInterface
from opendbc.car.landrover.values import CAR
from opendbc.car.structs import CarParams


@pytest.mark.parametrize("alpha_long", [False, True])
def test_rr2017_preserves_stock_acc_and_disables_actuation(alpha_long):
  candidate = CAR.RANGEROVER_VOGUE_2017
  fingerprint = gen_empty_fingerprint()
  cp = CarInterface.get_params(candidate, fingerprint, [], alpha_long, False, False)
  CarInterface.get_params_sp(cp, candidate, fingerprint, [], alpha_long, False)
  assert cp.pcmCruise and cp.radarUnavailable and cp.dashcamOnly
  assert not cp.openpilotLongitudinalControl and not cp.alphaLongitudinalAvailable
  assert cp.safetyConfigs[0].safetyModel == CarParams.SafetyModel.noOutput


@pytest.mark.parametrize("alpha_long", [False, True])
def test_defender_retains_configuration(alpha_long):
  candidate = CAR.LANDROVER_DEFENDER_2023
  fingerprint = gen_empty_fingerprint()
  cp = CarInterface.get_params(candidate, fingerprint, [], alpha_long, False, False)
  CarInterface.get_params_sp(cp, candidate, fingerprint, [], alpha_long, False)
  assert not cp.dashcamOnly and not cp.radarUnavailable
  assert cp.openpilotLongitudinalControl == alpha_long
  assert cp.safetyConfigs[0].safetyModel == CarParams.SafetyModel.landrover
