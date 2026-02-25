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

    #if any(be.type == ButtonType.lkas and be.pressed for be in CS.out.buttonEvents):
    #  CS.lkas_disabled = False

    return MadsDataSP(enable_mads, paused, CS.lkas_disabled)

  def update(self, CC: structs.CarControl, CC_SP: structs.CarControlSP, CS) -> None:
    self.mads = self.mads_status_update(CC, CC_SP, CS)


class MadsCarState(MadsCarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    self.init_lkas_disabled = False
    self.lkas_disabled = False

  @staticmethod
  def create_lkas_button_events(cur_btn: int, prev_btn: int,
                                buttons_dict: dict[int, structs.CarState.ButtonEvent.Type]) -> list[structs.CarState.ButtonEvent]:
    events: list[structs.CarState.ButtonEvent] = []

    if cur_btn == prev_btn:
      return events

    state_changes = [
      {"pressed": prev_btn != cur_btn},
    ]

    for change in state_changes:
      if change["pressed"]:
        events.append(structs.CarState.ButtonEvent(pressed=change["pressed"],
                                                   type=buttons_dict.get(cur_btn, ButtonType.unknown)))
    return events

  def update_mads(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]

    self.prev_lkas_button = self.lkas_button
    if self.CP.carFingerprint in EVA2_CARS:
      self.lkas_button = cp.vl["LKAS_BTN"]["LKAS_Btn_on"]
      ret.buttonEvents = self.create_lkas_button_events(self.lkas_button, self.prev_lkas_button, {1: ButtonType.lkas})
