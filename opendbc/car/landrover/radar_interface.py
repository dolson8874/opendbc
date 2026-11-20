import os
import time
from dataclasses import dataclass, field
from typing import NamedTuple

from opendbc.can.dbc import DBC
from opendbc.car import Bus
from opendbc.car.can_definitions import CanData
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.landrover.values import DBC as LANDROVER_DBC

RADAR_BUS = 1
SCAN_HEADER = 0x190
SCAN_TIMEOUT_NS = 200_000_000
SLOT_ADDRESSES = tuple(0x202 + 16 * slot for slot in range(40))
MEMBER2_ADDRESSES = frozenset(SLOT_ADDRESSES)


class SlotEstimate(NamedTuple):
  mono_time_ns: int
  range_est_m: float
  relative_speed_est_mps: float
  lateral_raw: int


@dataclass
class RadarScan:
  start_ns: int
  counter: int
  seen: set[int] = field(default_factory=set)
  member2: dict[int, tuple[int, bytes]] = field(default_factory=dict)
  corrupt: bool = False


class RadarScanDecoder:
  def __init__(self, dbc_name: str):
    self.messages = DBC(dbc_name).addr_to_msg
    self.current: RadarScan | None = None
    self.latest: RadarScan | None = None
    self.last_ns = 0

  def expire(self, mono_time_ns: int) -> None:
    if self.current is not None and mono_time_ns - self.current.start_ns > SCAN_TIMEOUT_NS:
      self.current = None
    if self.latest is not None and mono_time_ns - self.latest.start_ns > SCAN_TIMEOUT_NS:
      self.latest = None

  def update(self, can_packets: list[tuple[int, list[CanData]]]) -> None:
    for mono_time_ns, frames in can_packets:
      if mono_time_ns < self.last_ns:
        continue
      self.last_ns = mono_time_ns
      self.expire(mono_time_ns)
      for address, data, source in frames:
        if source != RADAR_BUS or address not in self.messages:
          continue
        length_ok = len(data) == self.messages[address].size
        if address == SCAN_HEADER:
          scan = self.current
          self.latest = None
          self.current = None
          if not length_ok:
            continue
          counter = data[0] & 0xF
          if (scan is not None and not scan.corrupt and len(scan.seen) == len(self.messages)
              and mono_time_ns > scan.start_ns and ((counter - scan.counter) & 0xF) == 1):
            self.latest = scan
          self.current = RadarScan(mono_time_ns, counter, {SCAN_HEADER})
          continue

        scan = self.current
        if scan is None or scan.corrupt:
          continue
        if not length_ok or (data[0] & 0xF) != scan.counter or address in scan.seen:
          scan.corrupt = True
          continue
        scan.seen.add(address)
        if address in MEMBER2_ADDRESSES:
          scan.member2[address] = (mono_time_ns, data)

  def slot_estimate(self, slot: int) -> SlotEstimate | None:
    if self.latest is None or not 0 <= slot < len(SLOT_ADDRESSES):
      return None
    address = SLOT_ADDRESSES[slot]
    mono_time_ns, data = self.latest.member2[address]
    sigs = self.messages[address].sigs
    distance = sigs["RANGE_EST_M"]
    speed = sigs["REL_SPEED_EST_MPS"]
    return SlotEstimate(mono_time_ns,
                        distance.get_raw_value(data) * distance.factor + distance.offset,
                        speed.get_raw_value(data) * speed.factor + speed.offset,
                        sigs["LATERAL_I51_U11_RAW"].get_raw_value(data))


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    dbc_name = LANDROVER_DBC[CP.carFingerprint].get(Bus.radar)
    self.decoder = RadarScanDecoder(dbc_name) if dbc_name is not None else None
    self.replay = 'REPLAY' in os.environ

  def update(self, can_packets):
    if self.decoder is not None:
      self.decoder.update(can_packets)
      if not can_packets and not self.replay:
        self.decoder.expire(time.monotonic_ns())
    # Range origin, lateral geometry and target lifetimes still require validation.
    # Preserve the vision-only lead until we can populate all RadarPoint fields.
    return super().update(can_packets)
