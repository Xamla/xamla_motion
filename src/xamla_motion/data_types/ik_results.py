# ik_results.py
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

import enum


@enum.unique
class ErrorCodes(enum.Enum):
    PROGRESS = 0
    SUCCESS = 1
    FAILURE = 99999
    SIGNAL_LOST = -9999
    PLANNING_FAILED = -1
    INVALID_MOTION_PLAN = -2
    MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE = -3
    CONTROL_FAILED = -4
    UNABLE_TO_AQUIRE_SENSOR_DATA = -5
    TIMED_OUT = -6
    PREEMPTED = -7
    START_STATE_IN_COLLISION = -10
    START_STATE_VIOLATES_PATH_CONSTRAINTS = -11
    GOAL_IN_COLLISION = -12
    GOAL_VIOLATES_PATH_CONSTRAINTS = -13
    GOAL_CONSTRAINTS_VIOLATED = -14
    INVALID_GROUP_NAME = -15
    INVALID_GOAL_CONSTRAINTS = -16
    INVALID_ROBOT_STATE = -17
    INVALID_LINK_NAME = -18
    INVALID_OBJECT_NAME = -19
    FRAME_TRANSFORM_FAILURE = -21
    COLLISION_CHECKING_UNAVAILABLE = -22
    ROBOT_STATE_STALE = -23
    SENSOR_INFO_STALE = -24
    NO_IK_SOLUTION = -31


class IkResults(object):
    """
    Class with hold result of a inverse kinematics query
    """

    def __init__(self, path, error_codes):
        self.__path = path
        self.__error_codes = [ErrorCodes(x.val) for x in error_codes]

    @property
    def path(self):
        """
        path : JointPath
            solutions from the inverse kinematics
        """
        return self.__path

    @property
    def error_codes(self):
        """
        error_code : List[ErrorCodes]
            error codes
        """
        return self.__error_codes

    @property
    def succeeded(self):
        if all([e == ErrorCodes.SUCCESS
                for e in self.__error_codes]):
            return True
        else:
            return False

    def __str__(self):
        return str(self.__error_codes)

    def __repr__(self):
        return self.__str__()
