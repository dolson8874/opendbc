import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.landrover.interface import CarInterface
from opendbc.car.landrover.values import CAR, LandroverFlags
from opendbc.car.structs import CarParams


@pytest.mark.parametrize("alpha_long", [False, True])
def test_rr2017_preserves_stock_acc_and_selects_rr_safety(alpha_long):
  candidate = CAR.RANGEROVER_VOGUE_2017
  fingerprint = gen_empty_fingerprint()
  cp = CarInterface.get_params(candidate, fingerprint, [], alpha_long, False, False)
  CarInterface.get_params_sp(cp, candidate, fingerprint, [], alpha_long, False)
  assert cp.pcmCruise and cp.radarUnavailable and not cp.dashcamOnly
  assert not cp.openpilotLongitudinalControl and not cp.alphaLongitudinalAvailable
  assert cp.safetyConfigs[0].safetyModel == CarParams.SafetyModel.landrover
  assert cp.safetyConfigs[0].safetyParam == LandroverFlags.RANGE_ROVER_2017


@pytest.mark.parametrize("reconfigure", [False, True])
def test_rr2017_uses_platform_torque_gain(reconfigure):
  candidate = CAR.RANGEROVER_VOGUE_2017
  cp = CarInterface.get_non_essential_params(candidate)

  if reconfigure:
    CarInterface.configure_torque_tune(candidate, cp.lateralTuning)

  tune = cp.lateralTuning.torque
  assert tune.kp == pytest.approx(0.85)
  assert tune.ki == pytest.approx(0.3)
  assert tune.kf == pytest.approx(1.0)
  assert tune.latAccelFactor == pytest.approx(3.1)
  assert tune.friction == pytest.approx(0.0)


@pytest.mark.parametrize("alpha_long", [False, True])
def test_defender_retains_configuration(alpha_long):
  candidate = CAR.LANDROVER_DEFENDER_2023
  fingerprint = gen_empty_fingerprint()
  cp = CarInterface.get_params(candidate, fingerprint, [], alpha_long, False, False)
  CarInterface.get_params_sp(cp, candidate, fingerprint, [], alpha_long, False)
  assert not cp.dashcamOnly and not cp.radarUnavailable
  assert cp.openpilotLongitudinalControl == alpha_long
  assert cp.safetyConfigs[0].safetyModel == CarParams.SafetyModel.landrover
