from opendbc.car.landrover.landrovercan import defender_crc

ADDRESS = 0x1FB
BUS = 1
TIMEOUT_NS = 200_000_000
LKAS_BUTTON_TIMEOUT_NS = 300_000_000


class CruiseHud:
  """Bench interpretation of the known Defender HUD combinations."""
  def __init__(self):
    self.counter: int | None = None
    self.received_ns = 0
    self.active: bool | None = None
    self.transport_valid = False

  def expire(self, now_ns: int) -> None:
    if self.counter is not None and now_ns - self.received_ns >= TIMEOUT_NS:
      self.counter = None
      self.active = None
      self.transport_valid = False

  def receive(self, data: bytes, time_ns: int) -> None:
    if self.counter is not None and time_ns < self.received_ns:
      return
    self.expire(time_ns)
    if len(data) != 8 or defender_crc(data[1:5]) != data[0] or data[2] & 0xE0 or any(data[4:]):
      self.active = None
      self.transport_valid = False
      return
    counter = data[1] & 0xF
    if self.counter is not None:
      delta = (counter - self.counter) & 0xF
      if not 1 <= delta <= 7:
        return
    self.counter = counter
    self.received_ns = time_ns
    self.transport_valid = True
    flags, mode = data[1] & 0xF0, data[2] & 7
    if flags == 0x30 and mode == 3:
      self.active = True
    elif (flags == 0x80 and mode == 0) or (flags == 0xC0 and mode == 6):
      # The cluster displays cruise as inactive in mode 6 with Off and Gray set.
      self.active = False
    else:
      self.active = None
