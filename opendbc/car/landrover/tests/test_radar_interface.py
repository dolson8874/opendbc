import pytest

from opendbc.can.dbc import DBC
from opendbc.car import Bus
from opendbc.car.landrover.interface import CarInterface
from opendbc.car.landrover.radar_interface import RadarInterface, RadarScanDecoder, SCAN_TIMEOUT_NS
from opendbc.car.landrover.values import CAR, DBC as LANDROVER_DBC

DEFENDER = CAR.LANDROVER_DEFENDER_2023
RADAR_DBC = LANDROVER_DBC[DEFENDER][Bus.radar]
MESSAGES = DBC(RADAR_DBC).addr_to_msg
START = 1_000_000_000
PERIOD = 50_000_000


def scan(counter):
  frames = []
  for address, msg in sorted(MESSAGES.items()):
    word = counter
    if address % 16 == 2 and address >= 0x202:
      word |= (address >> 4) << 28 | 2016 << 39 | 257 << 51
    frames.append((address, word.to_bytes(msg.size, 'little'), 1))
  return frames


def test_registered_defender_interface_preserves_empty_standard_tracks():
  assert CarInterface.RadarInterface is RadarInterface
  cp = CarInterface.get_non_essential_params(DEFENDER)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, DEFENDER)
  ri = CarInterface.RadarInterface(cp, cp_sp)
  for n in range(5):
    data = ri.update([(START + n * PERIOD, scan((15 + n) % 16))])
    if n < 4:
      assert data is None
    else:
      assert data is not None
      assert not data.points
      assert not any(data.errors.to_dict().values())
  assert not cp.openpilotLongitudinalControl
  estimate = ri.decoder.slot_estimate(0)
  assert estimate is not None
  assert estimate.mono_time_ns == START + 3 * PERIOD
  assert estimate.range_est_m == pytest.approx(3.2)
  assert estimate.relative_speed_est_mps == pytest.approx(-1.6)
  assert estimate.lateral_raw == 257
  assert ri.decoder.slot_estimate(39).range_est_m == pytest.approx(7.1)
  assert ri.decoder.slot_estimate(40) is None


@pytest.mark.parametrize("problem", ["other_bus", "short_dlc", "duplicate", "counter", "missing", "missing_header", "repeated_counter"])
def test_reject_incomplete_scan_and_recover(problem):
  decoder = RadarScanDecoder(RADAR_DBC)
  frames = scan(15)
  address, data, bus = frames[9]
  next_frames = scan(0)
  if problem == "other_bus":
    frames[9] = (address, data, 0)
  elif problem == "short_dlc":
    frames[9] = (address, data[:-1], bus)
  elif problem == "duplicate":
    frames.append(frames[9])
  elif problem == "counter":
    frames[9] = (address, bytes([data[0] ^ 1]) + data[1:], bus)
  elif problem == "missing":
    del frames[9]
  elif problem == "missing_header":
    del frames[0]
  elif problem == "repeated_counter":
    next_frames = scan(15)
  next_counter = 0 if problem == "repeated_counter" else 1
  decoder.update([(START, frames), (START + PERIOD, next_frames)])
  assert decoder.slot_estimate(0) is None
  decoder.update([(START + 2 * PERIOD, scan(next_counter))])
  assert decoder.slot_estimate(0) is not None


def test_scan_boundary_wrap_split_receive_and_expiry():
  decoder = RadarScanDecoder(RADAR_DBC)
  frames = scan(15)
  decoder.update([(START, frames[:8]), (START + 10_000_000, frames[8:]),
                  (START + PERIOD, scan(0)[:1])])
  sample = decoder.slot_estimate(0)
  assert sample is not None and sample.mono_time_ns == START + 10_000_000
  decoder.update([(START + SCAN_TIMEOUT_NS + 1, [(0x203, bytes(8), 0)])])
  assert decoder.latest is None
  assert decoder.slot_estimate(0) is None
  decoder.update([(START + SCAN_TIMEOUT_NS + PERIOD + 1, scan(1))])
  assert decoder.slot_estimate(0) is None
