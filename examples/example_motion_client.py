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
import signal
import functools
import asyncio


# function to shutdown asyncio properly
def shutdown(loop, reason):
    print('shutdown asyncio due to : {}'.format(reason), flush=True)
    tasks = asyncio.gather(*asyncio.Task.all_tasks(loop=loop),
                           loop=loop, return_exceptions=True)
    tasks.add_done_callback(lambda t: loop.stop())
    tasks.cancel()

    # Keep the event loop running until it is either destroyed or all
    # tasks have really terminated
    while not tasks.done() and not loop.is_closed():
        loop.run_forever()


# functions for supervised executation
async def next(stepped_motion_client):
    while True:
        await asyncio.sleep(0.1)
        if stepped_motion_client.state:
            stepped_motion_client.next()
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
    loop.add_signal_handler(signal.SIGTERM,
                            functools.partial(shutdown, loop, signal.SIGTERM))
    loop.add_signal_handler(signal.SIGINT,
                            functools.partial(shutdown, loop, signal.SIGINT))

    try:
        print('test MoveGroup class')
        print('----------------          move joints                 -------------------')
        loop.run_until_complete(
            move_group.move_joints(joint_path))
        print('----------------      move joints collision free      -------------------')
        loop.run_until_complete(
            move_group.move_joints_collision_free(joint_path))

        print('----------------        move joints supervised        -------------------')
        stepped_motion_client = move_group.move_joints_supervised(
            joint_path, 0.5)
        loop.run_until_complete(run_supervised(stepped_motion_client))

        print('----------------move joints collision free supervised -------------------')
        stepped_motion_client = move_group.move_joints_collision_free_supervised(
            joint_path, 0.5)
        loop.run_until_complete(run_supervised(stepped_motion_client))

        print('test EndEffector class')
        print('----------------           move poses                 -------------------')
        loop.run_until_complete(
            end_effector.move_poses_collision_free(cartesian_path))

        print('----------------        move poses collision free     -------------------')
        loop.run_until_complete(
            end_effector.move_poses_collision_free(cartesian_path))

        print('----------------          move poses supervised       -------------------')
        stepped_motion_client = end_effector.move_poses_supervised(
            cartesian_path, None, 0.5)
        loop.run_until_complete(run_supervised(stepped_motion_client))

        print('---------------- move poses collision free supervised -------------------')
        stepped_motion_client = end_effector.move_poses_collision_free_supervised(
            cartesian_path, None, 0.5)
        loop.run_until_complete(run_supervised(stepped_motion_client))

    finally:
        loop.close()


if __name__ == '__main__':
    main()
