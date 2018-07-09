#!/usr/bin/env python3

from data_types import *
from high_level_api import MoveGroup, EndEffector
from pyquaternion import Quaternion
import rospy
import asyncio

import pdb

rospy.init_node('test_motion_service')
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

ioloop = asyncio.get_event_loop()

print('test MoveGroup class')
print('----------------move joints collision free -------------------')

for i in range(0, 2):
    print('trajectory loop: ' + str(i))
    ioloop.run_until_complete(
        move_group.move_joints_collision_free(joint_path))

print('--------------------- move joints ----------------------------')

for i in range(0, 3):
    print('--- trajectory loop: ' + str(i) + ' -----')
    print('point1 10 percent of max velocity')
    ioloop.run_until_complete(move_group.move_joints(joint_path[0], 0.1))
    print('point2 50 percent of max velocity')
    ioloop.run_until_complete(move_group.move_joints(joint_path[1], 0.5))
    print('point3 100 percent of max velocity')
    ioloop.run_until_complete(move_group.move_joints(joint_path[2], 1.0))


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


ioloop.close()
