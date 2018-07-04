from __future__ import (absolute_import, division,
                        print_function)  # , unicode_literals)
# from future.builtins import *
from future.utils import raise_from, raise_with_traceback

from data_types import *
from motion_service import MotionService
import pdb

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

# print('---------create plan parameters------------')

# plan_parameters0 = motion_service.create_plan_parameters()
# print(plan_parameters0)
# plan_parameters1 = motion_service.create_plan_parameters(groups[0].name)
# print(plan_parameters1)
# plan_parameters2 = motion_service.create_plan_parameters(groups[0].name,
#                                                          groups[0].joint_set)
# print(plan_parameters2)

# max_velocities = ['1.0']*len(groups[0].joint_set)
# plan_parameters3 = motion_service.create_plan_parameters(groups[0].name,
#                                                          None,
#                                                          max_velocities,
#                                                          velocity_scaling=0.6)
# print(plan_parameters3)


print('---------create task space plan parameters------------')

t_plan_parameters0 = motion_service.create_task_space_plan_parameters()
print(t_plan_parameters0)
t_plan_parameters1 = motion_service.create_task_space_plan_parameters(
    list(end_effectors)[0])
print(t_plan_parameters1)

max_xyz_velocities = 1.0
t_plan_parameters3 = motion_service.create_task_space_plan_parameters(list(end_effectors)[0],
                                                                      max_xyz_velocities,
                                                                      velocity_scaling=0.6)
print(t_plan_parameters3)
