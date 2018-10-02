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
from xamla_motion.gripper_client import *
from xamla_motion.motion_service import SteppedMotionClient
from threading import Thread
from pyquaternion import Quaternion
import time
import asyncio

import pdb


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

    ioloop = asyncio.get_event_loop()

    async def print_Hallo():
        print('Hallo')

    try:
        print('test MoveGroup class')
        print('----------------move joints collision free -------------------')

        for i in range(0, 2):
            print('trajectory loop: ' + str(i))
            ioloop.run_until_complete(
                asyncio.wait([move_group.move_joints_collision_free(joint_path),
                              print_Hallo()]))

        print('--------------------- move joints ----------------------------')

        for i in range(0, 3):
            print('--- trajectory loop: ' + str(i) + ' -----')
            print('point1 10 percent of max velocity')
            ioloop.run_until_complete(
                move_group.move_joints(joint_path[0], 0.1))
            print('point2 50 percent of max velocity')
            ioloop.run_until_complete(
                move_group.move_joints(joint_path[1], 0.5))
            print('point3 100 percent of max velocity')
            ioloop.run_until_complete(
                move_group.move_joints(joint_path[2], 1.0))

        print('test EndEffector class')
        print('----------------move poses collision free -------------------')

        end_effector = EndEffector.from_end_effector_name(
            move_group.selected_end_effector)

        for i in range(0, 2):
            print('trajectory loop: ' + str(i))
            ioloop.run_until_complete(
                end_effector.move_poses_collision_free(cartesian_path))

        print('--------------------- move poses ----------------------------')

        for i in range(0, 3):
            print('--- trajectory loop: ' + str(i) + ' -----')
            print('point1 10 percent of max velocity')
            ioloop.run_until_complete(
                end_effector.move_poses(cartesian_path[0], 0.1))
            print('point2 50 percent of max velocity')
            ioloop.run_until_complete(
                end_effector.move_poses(cartesian_path[1], 0.5))
            print('point3 100 percent of max velocity')
            ioloop.run_until_complete(
                end_effector.move_poses(cartesian_path[2], 1.0))

        print('---------- wsg gripper -------------')

        properties = WeissWsgGripperProperties('wsg50')

        wsg_gripper = WeissWsgGripper(properties, move_group.motion_service)

        print('homeing')
        ioloop.run_until_complete(wsg_gripper.homing())

        print('move gripper')
        ioloop.run_until_complete(wsg_gripper.move(0.1, 0.05, 0.05, True))

        print('perform grasp')
        result = ioloop.run_until_complete(wsg_gripper.grasp(0.02, 0.1, 0.05))

        print('---------- common gripper -------------')
        properties1 = CommonGripperProperties('wsg50', 'xamla/wsg_driver')

        gripper = CommonGripper(properties1, move_group.motion_service)

        print('move')
        result1 = ioloop.run_until_complete(gripper.move(0.1, 0.005))

    finally:
        ioloop.close()

    # print('-----stepped motion client------')

    # def run_motion(motion_client):
    #     ioloop = asyncio.new_event_loop()
    #     asyncio.set_event_loop(ioloop)
    #     ioloop.run_until_complete(motion_client.moveJ_supervised(joint_trajectory,
    #                                                              0.1))
    #     ioloop.close()

    # stepped_motion_client = SteppedMotionClient()

    # thread = Thread(target=run_motion, args=(stepped_motion_client,))
    # thread.start()

    # time.sleep(2)
    # import pdb
    # count = 0
    # while stepped_motion_client.state != None:
    #     time.sleep(0.05)
    #     stepped_motion_client.next()

    #     if not (count % 100):
    #         print('progress {:5.2f} percent'.format(
    #             stepped_motion_client.state.progress))
    #     count += 1

    # thread.join()


if __name__ == '__main__':
    main()
