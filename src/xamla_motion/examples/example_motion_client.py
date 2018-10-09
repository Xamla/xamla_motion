# example_motion_client.py
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

from xamla_motion.data_types import Pose, CartesianPath, JointPath
from xamla_motion.motion_client import MoveGroup, EndEffector
from pyquaternion import Quaternion
import asyncio


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

    seed = move_group.get_current_joint_positions()
    joint_path = move_group.motion_service.query_inverse_kinematics_many(cartesian_path,
                                                                         move_group.default_plan_parameters,
                                                                         seed).path

    ioloop = asyncio.get_event_loop()

    try:
        print('test MoveGroup class')
        print('----------------move joints collision free -------------------')

        for i in range(0, 2):
            print('trajectory loop: ' + str(i))
            ioloop.run_until_complete(
                move_group.move_joints_collision_free(joint_path))

        print('test EndEffector class')
        print('----------------move poses collision free -------------------')

        for i in range(0, 2):
            print('trajectory loop: ' + str(i))
            ioloop.run_until_complete(
                end_effector.move_poses_collision_free(cartesian_path))

    finally:
        ioloop.close()

    if __name__ == '__main__':
        main()
