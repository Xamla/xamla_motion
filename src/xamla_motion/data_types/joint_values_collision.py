# joint_values_collision.py
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


class JointValuesCollisions(object):

    def __init__(self, index, error_code, error_message):
        self.__index = index
        self.__error_code = error_code
        self.__error_message = error_message

    @property
    def index(self):
        """
        index : int (readonly)
            value index where the collision occurs
        """
        return self.__index

    @property
    def error_code(self):
        """
        error_code : int (readonly)
            STATE_VALID = 1
            STATE_SELF_COLLISION = -1
            STATE_SCENE_COLLISION =-2
            INVALID_JOINTS = -11
            INVALID_MOVE_GROUP = -12
        """
        return self.__error_code

    @property
    def error_message(self):
        """
        error_message : str
            error message of state
        """
        return self.__error_message

    def __str__(self):
        return ('index_ ' + str(self.__index) +
                ' error: '+self.__error_message)

    def __repr__(self):
        return self.__str__()
