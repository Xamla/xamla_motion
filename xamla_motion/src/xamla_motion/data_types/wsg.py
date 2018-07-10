# wsg_command.py
#
# Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

#!/usr/bin/env python3

from enum import Enum, unique


@unique
class WsgCommand(Enum):
    stop = 100
    move = 101
    grasp = 102
    release = 103
    homing = 104
    acknowledge_error = 105


@unique
class WsgState(Enum):

    Idle = 0,
    Grasping = 1
    NoPartFound = 2
    PartLost = 3
    Holding = 4
    Releasing = 5
    Positioning = 6
    Error = 7
    Unknown = -1


class WsgResult(object):

    def __init__(self, state: WsgState, width: float,
                 force: float, status: str):
        self.__state = state
        self.__width = width
        self.__force = force
        self.__status = status

    @classmethod
    def from_wsg_command_action_result(cls, msg):
        status = msg.status

        return cls(status.grasping_id, status.width,
                   status.current_force, status.grasping_state)

    @property
    def state(self):
        return self.__state

    @property
    def width(self):
        return self.__width

    @property
    def force(self):
        return self.__force

    @property
    def status(self):
        return self.__status
