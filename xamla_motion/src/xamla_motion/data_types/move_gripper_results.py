# move_gripper_results.py
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

from control_msgs.msg import GripperCommandResult


class MoveGripperResults(object):

    def __init___(self, position: float, reached_goal: float,
                  stalled: bool, max_effort: float):
        self.__position = position
        self.__reached_goal = reached_goal
        self.__stalled = stalled
        self.__max_effort = max_effort

    @classmethod
    def from_gripper_command_action_result(cls, msg):
        result = msg.result
        return cls(result.position, result.reached_goal,
                   result.stalled, result.max_effort)

    @property
    def position(self):
        return self.__position

    @property
    def reached_goal(self):
        return self.__reached_goal

    @property
    def stalled(self):
        return self.__stalled

    @property
    def max_effort(self):
        return self.__max_effort
