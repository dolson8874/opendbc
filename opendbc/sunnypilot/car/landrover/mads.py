"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum
from collections import namedtuple

from opendbc.car import Bus, structs
from opendbc.car.landrover.values import EVA2_CARS

from opendbc.sunnypilot.mads_base import MadsCarStateBase
from opendbc.can.parser import CANParser

MadsDataSP = namedtuple("MadsDataSP",
                        ["enable_mads", "paused", "lkas_disabled"])

ButtonType = structs.CarState.ButtonEvent.Type


class MadsCarController:
  def __init__(self):
    self.mads = MadsDataSP(False, False, False)

  @staticmethod
  def mads_status_update(CC: structs.CarControl, CC_SP: structs.CarControlSP, CS) -> MadsDataSP:
    enable_mads = CC_SP.mads.available
    paused = CC_SP.mads.enabled and not CC.latActive

    if any(be.type == ButtonType.lkas and be.pressed for be in CS.out.buttonEvents):
      CS.lkas_disabled = False

    return MadsDataSP(enable_mads, paused, CS.lkas_disabled)

  def update(self, CC: structs.CarControl, CC_SP: structs.CarControlSP, CS) -> None:
    self.mads = self.mads_status_update(CC, CC_SP, CS)


class MadsCarState(MadsCarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    self.init_lkas_disabled = False
    self.lkas_disabled = False

  def get_lkas_button(self, cp):
    if self.CP.carFingerprint in EVA2_CARS:
      lkas_button = cp.vl["LKAS_BTN"]["LKAS_Btn_on"]
    else:
      lkas_button = 0

    return lkas_button

  def update_mads(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]

    self.prev_lkas_button = self.lkas_button
    self.lkas_button = self.get_lkas_button(cp)
