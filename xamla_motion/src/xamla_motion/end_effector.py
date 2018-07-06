# end_effector.py
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

from motion_service import MotionService
from move_group import MoveGroup

import asyncio


class EndEffector(object):

    def __init__(self, move_group, end_effector_name,
                 end_effector_link_name):

        if not is isinstance(move_group, MoveGroup):
            raise TypeError('move_group is not of expected'
                            ' type MoveGroup')

        self.__move_group = move_group
        self.__end_effector_name = str(end_effector_name)
        self.__end_effector_link_name = str(end_effector_link_name)
        self._m_service = move_group.motion_service

        async def move_pose(target, seed, velocity_scaling, acceleration_scaling, collision_check):
            pass
