from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from future.builtins import *
from future.utils import raise_from, raise_with_traceback

from MotionService import MotionServices
import pdb

motion_service = MotionServices()

print('----------------query move groups ---------------')
groups = motion_service.query_available_move_groups()

print(groups)

print('----------------query end effectors --------------')

end_effectors = motion_service.query_available_end_effectors()

print(end_effectors)

print('----------------query end effectors limits --------------')

end_effector_limits = motion_service.query_endeffector_limits('EE_manipulator')

print(end_effector_limits)

print('----------------query joint limits --------------')

joint_limits = motion_service.query_joint_limits(groups[0].joint_set)

print(joint_limits)

print('----------------query joint limits --------------')

joint_states = motion_service.query_joint_states(groups[0].joint_set)

print(joint_states)

pdb.set_trace()
