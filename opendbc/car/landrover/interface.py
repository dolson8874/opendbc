from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.landrover.carcontroller import CarController
from opendbc.car.landrover.carstate import CarState
from opendbc.car.landrover.values import CAR, LandroverFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def configure_torque_tune(candidate: str, tune: structs.CarParams.LateralTuning, steering_angle_deadzone_deg: float = 0.0):
    CarInterfaceBase.configure_torque_tune(candidate, tune, steering_angle_deadzone_deg)
    if candidate == CAR.RANGEROVER_VOGUE_2017:
      tune.torque.kp = 0.85

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "landrover"

    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 0.5

    ret.radarUnavailable = True
    ret.alphaLongitudinalAvailable = ret.radarUnavailable

    if alpha_long:
      ret.openpilotLongitudinalControl = True

    ret.steerControlType = structs.CarParams.SteerControlType.torque

    if ret.centerToFront == 0:
      ret.centerToFront = ret.wheelbase * 0.4

    if candidate == CAR.RANGEROVER_VOGUE_2017:
      ret.flags |= int(LandroverFlags.RANGE_ROVER_2017)
      ret.alphaLongitudinalAvailable = False
      ret.openpilotLongitudinalControl = False
      ret.pcmCruise = True
      CarInterface.configure_torque_tune(candidate, ret.lateralTuning)
      ret.steerActuatorDelay = 0.11
      ret.steerLimitTimer = 0.4
      ret.enableBsm = True

    elif candidate in (CAR.LANDROVER_DEFENDER_2023):
      ret.steerControlType = structs.CarParams.SteerControlType.angle
      ret.enableBsm = True

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.landrover, ret.flags)]

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, docs: bool) -> structs.CarParamsSP:

    stock_cp.enableBsm = True
    stock_cp.radarUnavailable = candidate == CAR.RANGEROVER_VOGUE_2017

    return ret
