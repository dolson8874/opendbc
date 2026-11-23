from types import SimpleNamespace

from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.landrover.interface import CarInterface
from opendbc.car.landrover.values import CAR


def test_neokii_rr_command_cadence_and_stock_acc_gating():
  candidate = CAR.RANGEROVER_VOGUE_2017
  fp = gen_empty_fingerprint()
  cp = CarInterface.get_params(candidate, fp, [], False, False, False)
  cp_sp = CarInterface.get_params_sp(cp, candidate, fp, [], False, False)
  controller = CarInterface.CarController({Bus.pt: 'landrover_rangerover_2017'}, cp, cp_sp)
  control = structs.CarControl(latActive=True)
  control.actuators.torque = 1
  state = structs.CarState(canValid=True)
  car_state = SimpleNamespace(out=state, lkas_disabled=False)
  published = []

  for frame in range(16):
    state.cruiseState.enabled = frame not in (0, 10)
    state.brakePressed = frame == 7
    state.leftBlinker = frame == 12
    _, messages = controller.update(control.as_reader(), structs.CarControlSP(), car_state, frame * 10_000_000)
    for addr, data, bus in messages:
      assert bus == 0
      torque = 1024 - (((data[3] & 7) << 8) | data[4]) if addr == 0x28F else 0
      published.append((frame, addr, torque))

  assert [frame for frame, addr, _ in published if addr == 0x1D8] == [2, 6, 10, 14]
  assert [frame for frame, addr, _ in published if addr == 0x3D4] == [0, 8]
  assert [(frame, torque) for frame, addr, torque in published if addr == 0x28F] == [(0, 0), (4, 12), (8, 0), (12, 0)]
