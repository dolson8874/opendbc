from types import SimpleNamespace

import pytest

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.landrover.cruise_hud import ADDRESS, BUS, TIMEOUT_NS
from opendbc.car.landrover.interface import CarInterface
from opendbc.car.landrover.landrovercan import defender_crc
from opendbc.car.landrover.values import CAR, LandroverFlags
from opendbc.safety.tests.libsafety import libsafety_py


def hud(counter, flags, mode, reserved=0):
  data = bytes([flags | counter, mode, 80, reserved, 0, 0, 0])
  return bytes([defender_crc(data[:4])]) + data


@pytest.fixture
def bench(monkeypatch):
  monkeypatch.setenv('LANDROVER_CRUISE_HUD_BENCH', '1')
  monkeypatch.setenv('REPLAY', '1')
  cp = CarInterface.get_non_essential_params(CAR.LANDROVER_DEFENDER_2023)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.LANDROVER_DEFENDER_2023)
  assert cp.flags & LandroverFlags.CRUISE_HUD_BENCH
  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(structs.CarParams.SafetyModel.landrover, int(cp.flags)) == 0
  safety.init_tests()
  return CarInterface(cp, cp_sp), safety


def feed(bench, us, address, src, data):
  interface, safety = bench
  safety.set_timer(us)
  safety.safety_rx_hook(libsafety_py.make_CANPacket(address, src, data))
  state, _ = interface.update([(us * 1000, [CanData(address, data, src)])])
  return state.cruiseState.enabled, state.blockPcmEnable, safety.get_controls_allowed()


def test_hud_continuity_prevents_legacy_res_edge(bench):
  assert feed(bench, 1, ADDRESS, BUS, hud(14, 0x80, 0)) == (False, True, False)
  assert feed(bench, 50_001, ADDRESS, BUS, hud(15, 0x30, 3)) == (True, False, True)
  assert feed(bench, 100_001, ADDRESS, BUS, hud(0, 0x30, 3)) == (True, True, True)
  assert feed(bench, 110_001, 0x102, 0, b'\0' * 8) == (True, True, True)
  assert feed(bench, 160_001, 0x102, 0, b'\0' * 4 + b'\4' + b'\0' * 3) == (True, True, True)
  assert feed(bench, 170_001, ADDRESS, BUS, hud(1, 0x80, 0)) == (False, True, False)


def test_timeout_duplicate_out_of_order_and_recovery_do_not_arm(bench):
  assert feed(bench, 1, ADDRESS, BUS, hud(0, 0x30, 3)) == (True, True, False)
  assert feed(bench, 50_001, ADDRESS, BUS, hud(1, 0x80, 0)) == (False, True, False)
  assert feed(bench, 100_001, ADDRESS, BUS, hud(2, 0x30, 3)) == (True, False, True)
  assert feed(bench, 180_001, ADDRESS, BUS, hud(2, 0x30, 3)) == (True, True, True)
  assert feed(bench, 220_001, ADDRESS, BUS, hud(1, 0x80, 0)) == (True, True, True)
  assert feed(bench, 300_000, 0x102, 0, bytes(8)) == (True, True, True)
  assert feed(bench, 100_001 + TIMEOUT_NS // 1000, 0x102, 0, bytes(8)) == (False, True, False)
  assert feed(bench, 310_001, ADDRESS, BUS, hud(0, 0x30, 3)) == (True, True, False)


@pytest.mark.parametrize('bad', [b'\0' * 8, hud(2, 0x30, 3, reserved=1), hud(2, 0x30, 0x23), hud(2, 0xC0, 6)])
def test_invalid_or_unknown_frame_disarms_both_paths(bench, bad):
  feed(bench, 1, ADDRESS, BUS, hud(0, 0x80, 0))
  assert feed(bench, 50_001, ADDRESS, BUS, hud(1, 0x30, 3)) == (True, False, True)
  assert feed(bench, 100_001, ADDRESS, BUS, bad) == (False, True, False)
  assert feed(bench, 150_001, ADDRESS, BUS, hud(3, 0x30, 3)) == (True, True, False)


def test_wrong_bus_and_panda_extended_frames(bench):
  assert feed(bench, 1, ADDRESS, 0, hud(0, 0x80, 0)) == (False, True, False)
  assert feed(bench, 30_001, ADDRESS, BUS, hud(1, 0x30, 3)[:-1]) == (False, True, False)
  interface, safety = bench
  state, _ = interface.update([(50_001_000, [CanData(ADDRESS, hud(1, 0x30, 3), 129)])])
  assert not state.cruiseState.enabled
  pkt = libsafety_py.make_CANPacket(ADDRESS, BUS, hud(0, 0x80, 0))
  pkt[0].extended = 1
  safety.safety_rx_hook(pkt)
  safety.safety_rx_hook(libsafety_py.make_CANPacket(ADDRESS, BUS, hud(1, 0x30, 3)))
  assert not safety.get_controls_allowed()


def test_bench_host_and_panda_deny_command_transmission(bench):
  interface, safety = bench
  assert interface.apply(SimpleNamespace(), SimpleNamespace())[1] == []
  safety.set_controls_allowed(True)
  for address, src in ((0x1F0, 1), (0x1F9, 1), (0x1BE, 0), (ADDRESS, BUS)):
    assert not safety.safety_tx_hook(libsafety_py.make_CANPacket(address, src, bytes(8)))
