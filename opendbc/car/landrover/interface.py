import os
import time

from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.landrover.carcontroller import CarController
from opendbc.car.landrover.carstate import CarState
from opendbc.car.landrover.values import CAR, LandroverFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def update(self, can_packets):
    if self.CP.flags & LandroverFlags.CRUISE_HUD_BENCH:
      now_ns = can_packets[-1][0] if 'REPLAY' in os.environ and can_packets else time.monotonic_ns()
      self.CS.update_cruise_hud(can_packets, now_ns)
    return super().update(can_packets)

  def apply(self, c, c_sp, now_nanos=None):
    if self.CP.flags & LandroverFlags.CRUISE_HUD_BENCH:
      # Keep the bench read-only even if an older Panda is connected.
      return structs.CarControl.Actuators(), []
    return super().apply(c, c_sp, now_nanos)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "landrover"

    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 0.5

    ret.radarUnavailable = True
    ret.alphaLongitudinalAvailable = not ret.radarUnavailable

    if alpha_long:
      ret.openpilotLongitudinalControl = True

    ret.openpilotLongitudinalControl = alpha_long and ret.alphaLongitudinalAvailable
    ret.pcmCruise = not ret.openpilotLongitudinalControl

    ret.steerControlType = structs.CarParams.SteerControlType.torque

    if ret.centerToFront == 0:
      ret.centerToFront = ret.wheelbase * 0.4

    """
    if candidate in (CAR.RANGEROVER_VOGUE_2017):
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.steerActuatorDelay = 0.11
      ret.deprecated.enableBsm = True
    """

    if candidate in (CAR.LANDROVER_DEFENDER_2023):
      ret.steerControlType = structs.CarParams.SteerControlType.angle
      ret.deprecated.enableBsm = True

    if ret.flags & LandroverFlags.FLEXRAY_HARNESS and os.environ.get('LANDROVER_CRUISE_HUD_BENCH') != '0':
      ret.flags |= int(LandroverFlags.CRUISE_HUD_BENCH)
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.landrover, ret.flags)]

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:

    stock_cp.deprecated.enableBsm = True
    stock_cp.radarUnavailable = False

    return ret
