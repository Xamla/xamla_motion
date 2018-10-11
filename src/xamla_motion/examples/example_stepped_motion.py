# example_ur5_and_wsg25.py
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

from xamla_motion.data_types import *
from xamla_motion.motion_client import MoveGroup, EndEffector
from xamla_motion.motion_service import SteppedMotionClient
from pyquaternion import Quaternion
import time
import asyncio


def main():
    move_group = MoveGroup()

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

    seed = move_group.motion_service.query_joint_states(
        move_group.default_plan_parameters.joint_set).positions
    joint_path = move_group.motion_service.query_inverse_kinematics_many(cartesian_path,
                                                                         move_group.default_plan_parameters,
                                                                         seed).path

    joint_path_cf = move_group.motion_service.plan_collision_free_joint_path(joint_path,
                                                                             move_group.default_plan_parameters)
    joint_trajectory = move_group.motion_service.plan_move_joints(joint_path_cf,
                                                                  move_group.default_plan_parameters)

    async def stepped_execution(stepped_motion_client):
        count = 0
        print('start_stepped_execution')
        while stepped_motion_client.state:
            time.sleep(0.1)
            stepped_motion_client.next()

            if not (count % 100):
                print('progress {:5.2f} percent'.format(
                    stepped_motion_client.state.progress))
            count += 1

    ioloop = asyncio.get_event_loop()

    try:
        print('-----stepped motion client------')

        stepped_motion_client = SteppedMotionClient()

        ioloop.run_until_complete(asyncio.wait(
            [stepped_motion_client.moveJ_supervised(joint_trajectory, 0.1),
             stepped_execution(stepped_motion_client)]))

    finally:
        ioloop.close()


if __name__ == '__main__':
    main()
