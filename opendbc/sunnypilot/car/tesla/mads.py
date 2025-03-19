"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs


class MadsCarController:
  def __init__(self):
    super().__init__()
    self.use_lka_mode = False

  def update(self, CC_SP: structs.CarControlSP):
    self.use_lka_mode = CC_SP.mads.available
