import pytest

from opendbc.can.packer import CANPacker
from opendbc.car.landrover.landrovercan import create_lkas_hud, create_rr_lkas_command
from opendbc.car.landrover.values import LandroverFlags, STATIC_MSGS
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety.libsafety_py import libsafety, make_CANPacket


@pytest.fixture
def rr():
  assert libsafety.set_safety_hooks(CarParams.SafetyModel.landrover, LandroverFlags.RANGE_ROVER_2017) == 0
  libsafety.init_tests()
  packer = CANPacker("landrover_rangerover_2017")
  counter = 0

  def rx(*, acc=False, brake=False, pedal=0, driver=0, cam=True):
    nonlocal counter
    signals = (
      ("EPS_04", {"STEER_TORQUE04": driver}),
      ("SPEED_02", {"SPEED02": 40}),
      ("ACCELATOR_DRIVER", {"ACCELATOR_DRIVER": pedal}),
      ("CRUISE_CONTROL", {"CRUISE_ON": acc, "DRIVER_BRAKE": brake}),
    )
    for name, values in signals:
      addr, payload, bus = packer.make_can_msg(name, 0, values)
      assert libsafety.safety_rx_hook(make_CANPacket(addr, bus, payload))
    if cam:
      addr, payload, _ = create_rr_lkas_command(counter, 0)
      assert libsafety.safety_rx_hook(make_CANPacket(addr, 2, payload))
      counter += 1

  return rx, packer


def tx(torque, **kwargs):
  address, payload, bus = create_rr_lkas_command(0, torque)
  return libsafety.safety_tx_hook(make_CANPacket(address, kwargs.get("bus", bus), kwargs.get("payload", payload)))


@pytest.mark.parametrize("pedal", ["brake", "gas"])
def test_acc_gate_and_pedal_release(rr, pedal):
  rx, _ = rr
  assert not tx(10)
  rx()
  rx(acc=True, cam=False)
  assert tx(-12)
  assert not tx(-25)
  assert tx(0)
  rx(acc=True, **({"brake": True} if pedal == "brake" else {"pedal": 1}))
  assert not tx(10)
  rx(acc=True)
  assert not tx(10)
  rx()
  rx(acc=True)
  assert tx(12)


def test_required_bus_freshness_and_checksum(rr):
  rx, _ = rr
  rx(cam=False)
  rx(acc=True, cam=False)
  assert not tx(12)
  rx()
  rx(acc=True)
  assert tx(12)
  assert not tx(12, bus=2)
  address, payload, _ = create_rr_lkas_command(0, 12)
  damaged = bytes([payload[0] ^ 1]) + payload[1:]
  assert not tx(12, payload=damaged)
  assert not libsafety.safety_rx_hook(make_CANPacket(address, 2, damaged))
  assert not tx(12)
  rx()
  rx(acc=True)
  assert tx(12)
  libsafety.set_timer(210_000)
  assert not tx(12)
  assert tx(0)


def test_driver_limits_unsafe_output_and_camera_forwarding(rr):
  rx, packer = rr
  for _ in range(6):
    rx(driver=-640)
  rx(acc=True, driver=-640)
  assert libsafety.get_torque_driver_max() == -640
  assert not tx(12)
  assert tx(-12)
  assert tx(0)

  assert libsafety.safety_fwd_hook(2, 0x28F) == -1
  assert libsafety.safety_fwd_hook(2, 0x3D4) == -1
  for left in range(4):
    for right in range(4):
      addr, payload, bus = create_lkas_hud(packer, left, right)
      assert libsafety.safety_tx_hook(make_CANPacket(addr, bus, payload))
  addr, bus, _, payload = STATIC_MSGS[0]
  assert libsafety.safety_tx_hook(make_CANPacket(addr, bus, payload))
  assert not libsafety.safety_tx_hook(make_CANPacket(addr, bus, bytes(8)))
  assert not libsafety.safety_tx_hook(make_CANPacket(0x1F0, 1, bytes(8)))
  assert libsafety.set_safety_hooks(CarParams.SafetyModel.landrover, 0) == 0
  assert not tx(0)


def test_stock_camera_on_actuation_bus_latches_relay_malfunction(rr):
  rx, _ = rr
  rx()
  rx(acc=True)
  assert tx(12)
  address, payload, bus = create_rr_lkas_command(0, 0)
  libsafety.safety_rx_hook(make_CANPacket(address, bus, payload))
  assert libsafety.get_relay_malfunction()
  assert not tx(0)
  assert not tx(12)
