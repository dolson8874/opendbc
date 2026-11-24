import pytest

from opendbc.can.packer import CANPacker
from opendbc.car.landrover.landrovercan import create_lkas_hud, create_rr_lkas_command
from opendbc.car.landrover.values import LandroverFlags, STATIC_MSGS
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety.libsafety_py import libsafety, make_CANPacket


@pytest.fixture
def rr(request):
  mads = getattr(request, 'param', False)
  flags = LandroverFlags.RANGE_ROVER_2017 | (LandroverFlags.RR2017_MADS if mads else 0)
  assert libsafety.set_safety_hooks(CarParams.SafetyModel.landrover, flags) == 0
  libsafety.init_tests()
  libsafety.set_mads_params(mads, False, False)
  if mads:
    libsafety.receive_rr2017_mads_heartbeat(True)
  packer = CANPacker("landrover_rangerover_2017")
  counter = 0

  def rx(*, acc=False, brake=False, pedal=0, driver=0, cam=True, gap=None, cancel=False):
    nonlocal counter
    signals = (
      ("EPS_04", {"STEER_TORQUE04": driver}),
      ("SPEED_02", {"SPEED02": 40}),
      ("ACCELATOR_DRIVER", {"ACCELATOR_DRIVER": pedal}),
      ("CRUISE_CONTROL", {"CRUISE_ON": acc, "DRIVER_BRAKE": brake}),
    )
    if gap is not None:
      signals = (("CR01", {"GAP": gap}), *signals)
    if mads and cancel is not None:
      signals = (*signals, ("CRUISE_ACC", {"CruiseCancelRequestCandidate": cancel}))
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
  rx(acc=True)
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


def test_stock_camera_repeats_without_extending_counter_liveness(rr):
  rx, _ = rr
  rx()
  rx(acc=True)
  for now in (40_000, 80_000, 120_000):
    libsafety.set_timer(now)
    rx(acc=True, cam=False)
    addr, data, _ = create_rr_lkas_command(1, 0)
    assert libsafety.safety_rx_hook(make_CANPacket(addr, 2, data))
    assert tx(12)
  rx(acc=True)
  assert tx(12)
  for now in (160_000, 200_000, 240_000, 280_000, 320_000, 360_000):
    libsafety.set_timer(now)
    rx(acc=True, cam=False)
    addr, data, _ = create_rr_lkas_command(2, 0)
    assert libsafety.safety_rx_hook(make_CANPacket(addr, 2, data))
  assert not tx(12)
  assert tx(0)


def test_repeated_camera_counter_with_changed_payload_disarms(rr):
  rx, _ = rr
  rx()
  rx(acc=True)
  assert tx(12)
  addr, data, _ = create_rr_lkas_command(1, 1)
  assert libsafety.safety_rx_hook(make_CANPacket(addr, 2, data))
  assert not tx(12)


@pytest.mark.parametrize("rr", [True], indirect=True)
@pytest.mark.parametrize("transition", ["brake", "cancel"])
def test_mads_retains_steering_with_gap_four_and_cancel_disarms_after_acc_off(rr, transition):
  rx, packer = rr
  rx(acc=True, gap=2)
  assert not tx(10)  # No observed ACC-off state since startup.
  rx(gap=2)
  rx(acc=True, gap=2)
  assert tx(10)

  libsafety.set_timer(70_000)
  rx(acc=True, brake=transition == "brake", gap=2, cancel=transition == "cancel")
  assert tx(10)
  libsafety.set_timer(140_000)
  rx(gap=2, brake=transition == "brake", cancel=transition == "cancel")
  assert tx(10)
  libsafety.set_timer(210_000)
  rx(gap=0, cancel=transition == "cancel")
  assert tx(10)
  libsafety.set_timer(280_000)
  rx(gap=2)
  assert tx(10)
  addr, payload, _ = packer.make_can_msg("CRUISE_ACC", 2, {"CruiseCancelRequestCandidate": 1})
  assert libsafety.safety_rx_hook(make_CANPacket(addr, 2, payload))
  assert tx(10)

  libsafety.set_timer(350_000)
  rx(gap=2, cancel=True)
  assert not libsafety.get_controls_allowed_lateral()
  assert not tx(10)
  assert tx(0)
  rx(gap=0)
  libsafety.receive_rr2017_mads_heartbeat(True)
  assert not tx(10)
  rx(acc=True, gap=2)
  assert tx(10)


@pytest.mark.parametrize("rr", [True], indirect=True)
def test_mads_first_cancel_when_acc_off_just_arrived_does_not_disarm(rr):
  rx, _ = rr
  rx(gap=2)
  rx(gap=2)  # Require a progressing camera counter before starting ACC.
  rx(acc=True, gap=2)
  assert tx(10)
  libsafety.set_timer(70_000)
  rx(gap=2, cancel=True)
  assert tx(10)  # ACC OFF and CANCEL in the same batch.
  for now in (140_000, 210_000):
    libsafety.set_timer(now)
    rx(gap=2, cancel=True)
    assert tx(10)  # Holding past the OFF confirmation window is not a new press.


@pytest.mark.parametrize("rr", [True], indirect=True)
def test_mads_gap_four_allows_engagement_and_lost_camera_or_gas_disarms(rr):
  rx, _ = rr
  rx(gap=2)
  rx(gap=2)  # Require a progressing camera counter before starting ACC.
  rx(acc=True, gap=2)
  assert tx(10)
  rx(gap=2, pedal=1)
  assert not tx(10)
  rx(gap=2)
  assert not tx(10)
  rx(acc=True, gap=2)
  assert tx(10)
  libsafety.set_timer(210_001)
  assert not tx(10)
  rx(acc=True, gap=2)
  assert not tx(10)


@pytest.mark.parametrize("rr", [True], indirect=True)
def test_mads_stale_cancel_input_disarms_even_with_other_inputs_fresh(rr):
  rx, _ = rr
  rx(gap=2)
  rx(gap=2)  # Require a progressing camera counter before starting ACC.
  rx(acc=True, gap=2)
  assert tx(10)
  for now in (70_000, 140_000, 210_000, 280_000, 350_000):
    libsafety.set_timer(now)
    rx(gap=2, cancel=None)
    assert tx(10)
  libsafety.set_timer(360_000)
  rx(gap=2, cancel=None)
  assert not tx(10)
  assert tx(0)
  rx(gap=2, cancel=True)
  assert not tx(10)


@pytest.mark.parametrize("rr", [True], indirect=True)
def test_mads_engages_and_stays_active_without_gap_messages(rr):
  rx, _ = rr
  rx()
  rx()  # Confirm camera counter progression before the ACC edge.
  rx(acc=True)
  assert tx(10)
  for time_us, gap in ((100_000, 0), (200_000, 2), (300_000, None), (400_001, None), (500_001, None), (600_001, None), (650_001, None)):
    libsafety.set_timer(time_us)
    libsafety.receive_rr2017_mads_heartbeat(True)
    rx(gap=gap)
    assert tx(10)
  # The driver can still cancel MADS while the GAP message is absent.
  libsafety.set_timer(720_001)
  rx(cancel=True)
  assert not tx(10)
  assert tx(0)


@pytest.mark.parametrize("rr", [True], indirect=True)
def test_mads_heartbeat_off_requires_a_new_acc_edge(rr):
  rx, _ = rr
  rx(gap=0)
  rx(gap=0)
  rx(acc=True, gap=0)
  assert tx(10)
  rx(brake=True, gap=0)
  assert tx(10)

  libsafety.receive_rr2017_mads_heartbeat(False)
  assert not libsafety.get_controls_allowed_lateral()
  assert not tx(10)
  libsafety.receive_rr2017_mads_heartbeat(True)
  assert not tx(10)

  libsafety.receive_rr2017_mads_heartbeat(False)
  rx(acc=True, gap=0)
  libsafety.receive_rr2017_mads_heartbeat(False)
  assert not tx(10)
  libsafety.receive_rr2017_mads_heartbeat(True)
  assert tx(10)


@pytest.mark.parametrize("rr", [True], indirect=True)
def test_mads_lost_heartbeat_disarms_with_fresh_can(rr):
  rx, _ = rr
  rx(gap=0)
  rx(gap=0)
  rx(acc=True, gap=0)
  for now in (100_000, 200_000, 300_000, 400_000, 499_999):
    libsafety.set_timer(now)
    rx(acc=True, gap=0)
    assert tx(10)

  libsafety.set_timer(500_000)
  assert tx(0)
  assert not tx(10)
  libsafety.receive_rr2017_mads_heartbeat(True)
  rx(acc=True, gap=0)
  assert not tx(10)
  rx(gap=0)
  rx(acc=True, gap=0)
  assert tx(10)
