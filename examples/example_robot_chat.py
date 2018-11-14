# example_robot_chat.py
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

import asyncio

from pyquaternion import Quaternion
from xamla_motion.data_types import CartesianPath, Pose
from xamla_motion.motion_client import MoveGroup
from xamla_motion.robot_chat_client import (RobotChatClient,
                                            RobotChatSteppedMotion)
from xamla_motion.utility import register_asyncio_shutdown_handler


def main():
    # create move group instance
    move_group = MoveGroup()
    # get default endeffector of the movegroup
    end_effector = move_group.get_end_effector()

    # create cartesian path and equivalent joint path
    t1 = [0.502522, 0.2580, 0.3670]
    q1 = Quaternion(w=0.304389, x=0.5272, y=0.68704, z=0.39666)

    t2 = [0.23795, 0.46845, 0.44505]
    q2 = Quaternion(w=0.212097, x=0.470916, y=0.720915, z=0.462096)

    t3 = [0.6089578, 0.3406782, 0.208865]
    q3 = Quaternion(w=0.231852, x=0.33222, y=0.746109, z=0.528387)

    pose_1 = Pose(t1, q1)
    pose_2 = Pose(t2, q2)
    pose_3 = Pose(t3, q3)

    cartesian_path = CartesianPath([pose_1, pose_2, pose_3])

    stepped_client = end_effector.move_poses_supervised(cartesian_path,
                                                        collision_check=True)

    robot_chat = RobotChatClient()

    robot_chat_stepped_motion = RobotChatSteppedMotion(robot_chat,
                                                       stepped_client,
                                                       move_group.name)

    loop = asyncio.get_event_loop()
    register_asyncio_shutdown_handler(loop)

    try:
        loop.run_until_complete(
            robot_chat_stepped_motion.handle_stepwise_motions())
    finally:
        loop.close()


if __name__ == '__main__':
    main()
