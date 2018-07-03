from __future__ import (absolute_import, division,
                        print_function)  # , unicode_literals)
# from future.builtins import *
from future.utils import raise_from, raise_with_traceback

from data_types import *
from motion_service import MotionServices
import pdb

motion_service = MotionServices()

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

pose = motion_service.query_pose(
    groups[0].name, joint_states.positions)

print(pose)

# print('---------query collision free joint path------')

# joint_states1 = JointStates(joint_states.positions+0.1)
# joint_path = JointPath(joint_states.joint_set, [joint_states.positions,
#                                                 joint_states1.positions])
# rjoint_path = motion_service.query_collision_free_joint_path(groups[0].name,
#                                                              joint_path)

# print(rjoint_path)


# print('---------query collision free joint path------')

# max_velocities = [1.0] * len(joint_path.joint_set)
# max_accelerations = [0.2] * len(joint_path.joint_set)
# max_deviation = 0.005

# joint_trajectory = motion_service.query_joint_trajectory(joint_path,
#                                                          max_velocities,
#                                                          max_accelerations,
#                                                          max_deviation,
#                                                          20.0)

# print(joint_trajectory)
