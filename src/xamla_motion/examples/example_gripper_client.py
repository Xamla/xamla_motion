# example_gripper_client.py
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

from xamla_motion.data_types import MoveGripperResult, WsgCommand, WsgResult
from xamla_motion.motion_client import MoveGroup
from xamla_motion.gripper_client import WeissWsgGripperProperties, CommonGripperProperties
from xamla_motion.gripper_client import WeissWsgGripper, CommonGripper
import asyncio


def main():
    ioloop = asyncio.get_event_loop()

    # create instance of movegroup to provide motion services
    move_group = MoveGroup()

    try:
        print('---------- wsg gripper -------------')

        # create instance of wsg gripper by name
        properties = WeissWsgGripperProperties('wsg50')

        wsg_gripper = WeissWsgGripper(properties, move_group.motion_service)

        print('homeing')
        ioloop.run_until_complete(wsg_gripper.homing())

        print('move gripper')
        ioloop.run_until_complete(wsg_gripper.move(0.1, 0.05, 0.05, True))

        print('perform grasp')
        result = ioloop.run_until_complete(wsg_gripper.grasp(0.02, 0.1, 0.05))

        print('---------- common gripper -------------')
        # create instance of common gripper by name and driver rosnode name
        properties1 = CommonGripperProperties('wsg50', 'xamla/wsg_driver')

        gripper = CommonGripper(properties1, move_group.motion_service)

        print('move')
        result1 = ioloop.run_until_complete(gripper.move(0.1, 0.005))

    finally:
        ioloop.close()


if __name__ == '__main__':
    main()
