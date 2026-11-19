import pytest

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.landrover.cruise_hud import ADDRESS, BUS, LKAS_BUTTON_TIMEOUT_NS, TIMEOUT_NS
from opendbc.car.landrover.interface import CarInterface
from opendbc.car.landrover.landrovercan import create_lkas_command_defender, defender_crc
from opendbc.car.landrover.values import CAR, LandroverFlags
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from opendbc.safety.tests.libsafety import libsafety_py


def hud(counter, flags, mode, reserved=0):
  data = bytes([flags | counter, mode, 80, reserved, 0, 0, 0])
  return bytes([defender_crc(data[:4])]) + data


@pytest.fixture
def bench(monkeypatch):
  monkeypatch.delenv('LANDROVER_CRUISE_HUD_BENCH', raising=False)
  monkeypatch.setenv('REPLAY', '1')
  cp = CarInterface.get_non_essential_params(CAR.LANDROVER_DEFENDER_2023)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.LANDROVER_DEFENDER_2023)
  assert cp.flags & LandroverFlags.CRUISE_HUD_BENCH
  safety = libsafety_py.libsafety
  config = cp.safetyConfigs[0]
  assert safety.set_safety_hooks(config.safetyModel.raw, config.safetyParam) == 0
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


@pytest.mark.parametrize('bad', [b'\0' * 8, hud(2, 0x30, 3, reserved=1), hud(2, 0x30, 0x23), hud(2, 0xC0, 0)])
def test_invalid_or_unknown_frame_disarms_both_paths(bench, bad):
  feed(bench, 1, ADDRESS, BUS, hud(0, 0x80, 0))
  assert feed(bench, 50_001, ADDRESS, BUS, hud(1, 0x30, 3)) == (True, False, True)
  assert feed(bench, 100_001, ADDRESS, BUS, bad) == (False, True, False)
  assert feed(bench, 150_001, ADDRESS, BUS, hud(3, 0x30, 3)) == (True, True, False)


def test_hud_mode6_inactive_preserves_manual_lkas_until_input_expires(bench):
  interface, safety = bench
  interface.CP.alternativeExperience = ALTERNATIVE_EXPERIENCE.ENABLE_MADS
  safety.mads_apply_alternative_experience(ALTERNATIVE_EXPERIENCE.ENABLE_MADS)
  assert feed(bench, 1, ADDRESS, BUS, hud(0, 0xC0, 6)) == (False, True, False)
  assert not safety.get_controls_allowed_lateral()
  feed(bench, 5_001, 0x24, 0, bytes(8))
  feed(bench, 10_001, 0x24, 0, bytes(7) + b'\x20')
  assert safety.get_controls_allowed_lateral()
  assert feed(bench, 50_001, ADDRESS, BUS, hud(1, 0xC0, 6)) == (False, False, False)
  assert safety.get_controls_allowed_lateral()

  address, data, bus = create_lkas_command_defender(interface.CC.packer, True, True, 0, 0)
  packet = libsafety_py.make_CANPacket(address, bus, data)
  assert safety.safety_tx_hook(packet)
  safety.set_timer(50_001 + TIMEOUT_NS // 1000)
  assert not safety.safety_tx_hook(packet)
  assert not safety.get_controls_allowed_lateral()


def test_manual_lkas_is_separate_from_cruise_and_requires_fresh_button(bench):
  interface, safety = bench
  interface.CP.alternativeExperience = ALTERNATIVE_EXPERIENCE.ENABLE_MADS
  safety.mads_apply_alternative_experience(ALTERNATIVE_EXPERIENCE.ENABLE_MADS)
  button_on = bytes(7) + b'\x20'
  feed(bench, 1, ADDRESS, BUS, hud(0, 0xC0, 6))
  feed(bench, 10_001, 0x24, 0, button_on)
  assert not safety.get_controls_allowed_lateral() and not interface.CS.out.cruiseState.available
  feed(bench, 20_001, 0x24, 0, bytes(8))
  assert feed(bench, 50_001, ADDRESS, BUS, hud(1, 0x30, 3)) == (True, False, True)
  assert not safety.get_controls_allowed_lateral()
  feed(bench, 60_001, 0x24, 0, button_on)
  assert safety.get_controls_allowed_lateral() and interface.CS.out.cruiseState.available

  assert feed(bench, 100_001, ADDRESS, BUS, hud(2, 0xC0, 0)) == (False, True, False)
  assert safety.get_controls_allowed_lateral() and interface.CS.out.cruiseState.available
  address, data, bus = create_lkas_command_defender(interface.CC.packer, True, True, 0, 0)
  packet = libsafety_py.make_CANPacket(address, bus, data)
  assert safety.safety_tx_hook(packet)
  feed(bench, 110_001, 0x24, 0, bytes(8))
  assert not safety.get_controls_allowed_lateral() and not interface.CS.out.cruiseState.available
  assert not safety.safety_tx_hook(packet)

  feed(bench, 120_001, 0x24, 0, button_on)
  for counter, us in enumerate((150_001, 250_001, 350_001), start=3):
    feed(bench, us, ADDRESS, BUS, hud(counter, 0xC0, 0))
  assert safety.get_controls_allowed_lateral() and interface.CS.out.cruiseState.available
  feed(bench, 120_001 + LKAS_BUTTON_TIMEOUT_NS // 1000, ADDRESS, BUS, hud(6, 0xC0, 0))
  assert not safety.get_controls_allowed_lateral() and not interface.CS.out.cruiseState.available
  feed(bench, 430_001, 0x24, 0, button_on)
  assert not safety.get_controls_allowed_lateral() and not interface.CS.out.cruiseState.available


@pytest.mark.parametrize('brake_mode', [ALTERNATIVE_EXPERIENCE.MADS_DISENGAGE_LATERAL_ON_BRAKE,
                                       ALTERNATIVE_EXPERIENCE.MADS_PAUSE_LATERAL_ON_BRAKE])
def test_manual_lkas_brake_and_hud_fault_remain_fail_closed(bench, brake_mode):
  interface, safety = bench
  settings = ALTERNATIVE_EXPERIENCE.ENABLE_MADS | brake_mode
  interface.CP.alternativeExperience = settings
  safety.mads_apply_alternative_experience(settings)
  feed(bench, 1, ADDRESS, BUS, hud(0, 0xC0, 0))
  feed(bench, 10_001, 0x24, 0, bytes(8))
  button_on = bytes(7) + b'\x20'
  feed(bench, 20_001, 0x24, 0, button_on)
  assert safety.get_controls_allowed_lateral()
  feed(bench, 30_001, 0x84, 0, bytes(2) + b'\x40' + bytes(5))
  assert not safety.get_controls_allowed_lateral()
  feed(bench, 40_001, 0x24, 0, bytes(8))
  feed(bench, 50_001, 0x84, 0, bytes(8))
  assert not safety.get_controls_allowed_lateral()
  feed(bench, 60_001, 0x24, 0, button_on)
  assert safety.get_controls_allowed_lateral()
  feed(bench, 70_001, ADDRESS, BUS, bytes(8))
  assert not safety.get_controls_allowed_lateral() and not interface.CS.out.cruiseState.available
  feed(bench, 80_001, ADDRESS, BUS, hud(1, 0xC0, 0))
  feed(bench, 90_001, 0x24, 0, button_on)
  assert not safety.get_controls_allowed_lateral() and not interface.CS.out.cruiseState.available


def test_hud_mode6_inactive_is_a_known_cruise_transition(bench):
  assert feed(bench, 1, ADDRESS, BUS, hud(0, 0xC0, 6)) == (False, True, False)
  assert feed(bench, 50_001, ADDRESS, BUS, hud(1, 0x30, 3)) == (True, False, True)
  assert feed(bench, 100_001, ADDRESS, BUS, hud(2, 0xC0, 6)) == (False, True, False)


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


def test_hud_cruise_keeps_existing_lkas_and_hud_transmission(bench):
  interface, safety = bench
  feed(bench, 1, ADDRESS, BUS, hud(0, 0x80, 0))
  for us in range(1_001, 7_001, 1_000):
    feed(bench, us, 0x11, 0, bytes(4) + (9000).to_bytes(2, 'big') + bytes(2))
  feed(bench, 50_001, ADDRESS, BUS, hud(1, 0x30, 3))

  control = structs.CarControl(enabled=True, latActive=True, actuators={'steeringAngleDeg': 0.09})
  _, sends = interface.apply(control.as_reader(), structs.CarControlSP(), 50_001_000)
  assert [(address, bus) for address, _, bus in sends] == [(0x1F0, 1), (0x1F9, 1)]
  steer_data = sends[0][1]
  assert steer_data[3] & 0x80
  assert int.from_bytes(steer_data[3:5], 'big') & 0x3FFF != 9000
  for address, data, bus in sends:
    assert safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, data))
  assert not safety.safety_tx_hook(libsafety_py.make_CANPacket(ADDRESS, BUS, hud(1, 0x30, 3)))

  addr, data, src = create_lkas_command_defender(interface.CC.packer, True, True, 100, 1)
  assert not safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, src, data))


def test_lkas_rejects_expired_cruise_input_without_waiting_for_rx(bench):
  interface, safety = bench
  feed(bench, 1, ADDRESS, BUS, hud(0, 0x80, 0))
  feed(bench, 50_001, ADDRESS, BUS, hud(1, 0x30, 3))
  control = structs.CarControl(enabled=True, latActive=True).as_reader()
  _, sends = interface.apply(control, structs.CarControlSP(), 50_001_000)
  address, data, bus = sends[0]
  packet = libsafety_py.make_CANPacket(address, bus, data)
  assert safety.safety_tx_hook(packet)
  safety.set_timer(50_001 + TIMEOUT_NS // 1000)
  assert not safety.safety_tx_hook(packet)
  assert not safety.get_controls_allowed()

  addr, data, src = create_lkas_command_defender(interface.CC.packer, False, False, 0, 1)
  assert safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, src, data))


@pytest.mark.parametrize(('setting', 'expected'), [('0', False), ('1', True), ('invalid', True)])
def test_flexray_config_override(monkeypatch, setting, expected):
  monkeypatch.setenv('LANDROVER_CRUISE_HUD_BENCH', setting)
  cp = CarInterface.get_non_essential_params(CAR.LANDROVER_DEFENDER_2023)
  assert bool(cp.flags & LandroverFlags.CRUISE_HUD_BENCH) == expected
  assert cp.safetyConfigs[0].safetyParam == cp.flags


def test_non_flexray_keeps_legacy_config(monkeypatch):
  monkeypatch.delenv('LANDROVER_CRUISE_HUD_BENCH', raising=False)
  cp = CarInterface._get_params(structs.CarParams(), CAR.LANDROVER_DEFENDER_2023, {}, [], False, False, False)
  assert not cp.flags & LandroverFlags.CRUISE_HUD_BENCH
  assert cp.safetyConfigs[0].safetyParam == cp.flags
