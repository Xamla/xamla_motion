# example_motion_services.py
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

from ..data_types import *
from ..motion_service import MotionService
from pyquaternion import Quaternion
import rospy


import asyncio
import pdb

rospy.init_node('test_motion_service')

motion_service = MotionService()

print('----------------query move groups---------------')
groups = motion_service.query_available_move_groups()

print(groups)

print('----------------query end effectors--------------')

end_effectors = motion_service.query_available_end_effectors()

print(end_effectors)

print('----------------query end effectors limits--------------')

end_effector_limits = motion_service.query_endeffector_limits(
    list(end_effectors)[0])

print(end_effector_limits)

print('----------------query joint limits--------------')

joint_limits = motion_service.query_joint_limits(groups[0].joint_set)

print(joint_limits)

print('----------------query joint states--------------')

joint_states = motion_service.query_joint_states(groups[0].joint_set)

print(joint_states)

print('----------------query Pose --------------')

joint_states1 = JointStates(joint_states.positions+0.1)
pose = motion_service.query_pose(
    groups[0].name, joint_states.positions)
pose1 = motion_service.query_pose(
    groups[0].name, joint_states1.positions)

print(pose)

# print('---------query collision free joint path------')


# joint_path = JointPath(joint_states.joint_set, [joint_states.positions,
#                                                 joint_states1.positions])
# rjoint_path = motion_service.query_collision_free_joint_path(groups[0].name,
#                                                              joint_path)

# print(rjoint_path)


# print('---------query joint trajectory------')

# max_velocities = [1.0] * len(joint_path.joint_set)
# max_accelerations = [0.2] * len(joint_path.joint_set)
# max_deviation = 0.005

# joint_trajectory = motion_service.query_joint_trajectory(joint_path,
#                                                          max_velocities,
#                                                          max_accelerations,
#                                                          max_deviation,
#                                                          20.0)

# print(joint_trajectory)

# print('---------query task space trajectory------')

# max_xyz_velocities = 1.0
# max_xyz_accelerations = 0.2
# max_angular_velocities = 0.2
# max_angular_accelerations = 0.1
# ik_jump_threshold = 1.2
# max_deviation = 0.005
# collision_check = True
# dt = 125

# cartesian_path = CartesianPath([pose, pose1])

# joint_trajectory1 = motion_service.query_task_space_trajectory(list(end_effectors)[0],
#                                                                cartesian_path,
#                                                                joint_states.positions,
#                                                                max_xyz_velocities,
#                                                                max_xyz_accelerations,
#                                                                max_angular_velocities,
#                                                                max_angular_accelerations,
#                                                                ik_jump_threshold,
#                                                                max_deviation,
#                                                                collision_check,
#                                                                dt)

# print(joint_trajectory1)


# print('---------query joint path collisions------')


# joint_path = JointPath(joint_states.joint_set, [joint_states.positions,
#                                                 joint_states1.positions])
# rjoint_path = motion_service.query_collision_free_joint_path(groups[0].name,
#                                                              joint_path)

# collisions = motion_service.query_joint_path_collisions(groups[0].name,
#                                                         joint_path)

# collisions1 = motion_service.query_joint_path_collisions(groups[0].name,
#                                                          rjoint_path)

# print(collisions)

print('---------create plan parameters------------')

plan_parameters0 = motion_service.create_plan_parameters()
print(plan_parameters0)
plan_parameters1 = motion_service.create_plan_parameters(groups[0].name)
print(plan_parameters1)
plan_parameters2 = motion_service.create_plan_parameters(groups[0].name,
                                                         groups[0].joint_set)
print(plan_parameters2)

max_velocities = ['1.0']*len(groups[0].joint_set)
max_velocities[3] = None
print(max_velocities)
plan_parameters3 = motion_service.create_plan_parameters(groups[0].name,
                                                         None,
                                                         max_velocities,
                                                         velocity_scaling=0.6)
print(plan_parameters3)


# print('---------create task space plan parameters------------')

# t_plan_parameters0 = motion_service.create_task_space_plan_parameters()
# print(t_plan_parameters0)
# t_plan_parameters1 = motion_service.create_task_space_plan_parameters(
#     list(end_effectors)[0])
# print(t_plan_parameters1)

# max_xyz_velocities = 1.0
# t_plan_parameters3 = motion_service.create_task_space_plan_parameters(list(end_effectors)[0],
#                                                                       max_xyz_velocities,
#                                                                       velocity_scaling=0.6)
# print(t_plan_parameters3)


# print('---------plan move pose linear------------')

# cartesian_path = CartesianPath.from_start_stop_point(pose, pose1)
# trajectory = motion_service.plan_move_pose_linear(cartesian_path,
#                                                   joint_states.positions,
#                                                   t_plan_parameters3)
# print(trajectory)


# print('---------plan move joints------------')

# joint_path = JointPath.from_start_stop_point(joint_states.positions,
#                                              joint_states1.positions)
# trajectory = motion_service.plan_move_joints(joint_path,
#                                             plan_parameters3)
# print(trajectory)


# print('--------- plan cartesian path------------')

# cartesian_path = CartesianPath.from_start_stop_point(pose, pose1)

# path = motion_service.plan_cartesian_path(cartesian_path,
#                                           plan_parameters3)


# print(path)


print('--------- get current joint values ------------')

positions = motion_service.get_current_joint_values(plan_parameters3.joint_set)
print(positions)


print('---------- move pose ------------------')

t1 = [0.502522, 0.2580, 0.3670]
q1 = Quaternion(w=0.304389, x=0.5272, y=0.68704, z=0.39666)

t2 = [0.23795, 0.46845, 0.44505]
q2 = Quaternion(w=0.212097, x=0.470916, y=0.720915, z=0.462096)

pose_l = Pose(t1, q1)
pose_r = Pose(t2, q2)

ioloop = asyncio.get_event_loop()

for i in range(0, 10):
    ioloop.run_until_complete(motion_service.move_pose(pose_l,
                                                       '',
                                                       plan_parameters3))
    ioloop.run_until_complete(motion_service.move_pose(pose_r,
                                                       '',
                                                       plan_parameters3))

ioloop.close()
