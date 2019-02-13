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

import asyncio

from pyquaternion import Quaternion
from xamla_motion.data_types import CartesianPath, JointPath, Pose
from xamla_motion.motion_client import EndEffector, MoveGroup
from xamla_motion.utility import register_asyncio_shutdown_handler
from xamla_motion import MoveJointsCollisionFreeOperation, MoveCartesianCollisionFreeOperation

# functions for supervised executation


async def next(stepped_motion_client):
    while True:
        await asyncio.sleep(0.1)
        if stepped_motion_client.state:
            stepped_motion_client.step()
            print('progress {:5.2f} percent'.format(
                stepped_motion_client.state.progress))


async def run_supervised(stepped_motion_client):
    print('start supervised execution')

    task_next = asyncio.ensure_future(next(stepped_motion_client))

    await stepped_motion_client.action_done_future
    task_next.cancel()

    print('finished supervised execution')


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

    joint_path = end_effector.inverse_kinematics_many(cartesian_path,
                                                      False).path

    loop = asyncio.get_event_loop()
    register_asyncio_shutdown_handler(loop)

    async def example_moves():
        print('test MoveGroup class')
        print('----------------          move joints                 -------------------')
        move_joints = move_group.move_joints(joint_path)
        move_joints = move_joints.with_velocity_scaling(0.1)

        move_joints_plan = move_joints.plan()

        await move_joints_plan.execute_async()

        print('----------------        move joints supervised        -------------------')
        stepped_motion_client = move_joints_plan.execute_supervised()
        await run_supervised(stepped_motion_client)

        print('----------------      move joints collision free      -------------------')
        move_joints_cf = MoveJointsCollisionFreeOperation(
            move_joints.to_args())

        await move_joints_cf.plan().execute_async()

        print('test EndEffector class')
        print('----------------          move cartesian               -------------------')
        move_cartesian = end_effector.move_cartesian(cartesian_path)
        move_cartesian = move_cartesian.with_velocity_scaling(0.4)

        move_cartesian_plan = move_cartesian.plan()

        await move_cartesian_plan.execute_async()

        print('----------------      move cartesian supervised        -------------------')
        stepped_motion_client = move_cartesian_plan.execute_supervised()
        await run_supervised(stepped_motion_client)

        print('----------------    move cartesian collision free      -------------------')
        move_cartesian_cf = MoveCartesianCollisionFreeOperation(
            move_cartesian.to_args())

        await move_cartesian_cf.plan().execute_async()

    try:
        loop.run_until_complete(example_moves())
    finally:
        loop.close()


if __name__ == '__main__':
    main()
