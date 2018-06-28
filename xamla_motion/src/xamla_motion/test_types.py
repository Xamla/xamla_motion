#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function)  # , unicode_literals)
#from future.builtins import *

from data_types import *

import pdb
import numpy as np
from pyquaternion import Quaternion

tmp0 = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
values = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
print(type(tmp0))
tmp1 = JointSet(tmp0)

tmp2 = JointValues(tmp1, values)
print(tmp2)

tmp3 = tmp2.reorder(
    JointSet(['Joint2', 'Joint1', 'Joint4', 'Joint3', 'Joint6', 'Joint5']))

tmp4 = JointValues(JointSet(['Joint2', 'Joint5', 'Joint1']), values[1:-2])

tmp5 = JointLimits(tmp1, values, values, values, values)

print(4/tmp2)
print((1, 2, 3, 4, 10, 6)/tmp2)
print(tmp2/4)
print(tmp2/(1, 2, 3, 10, 5, 1))
print(tmp2/tmp4)
print(tmp4/tmp2)

transformation_matrix = np.eye((4), dtype=float)
transformation_matrix[:-1, -1] = np.array([1., 2., 3.]).T
print(transformation_matrix)
pose = Pose(transformation_matrix)
print(pose)
transformation_matrix[:-1, -1] = np.array([2., 3., 4.]).T
print(transformation_matrix)
pose1 = Pose(transformation_matrix)
print(pose1)
pose2 = Pose(np.array([1., 2., 3.], dtype=float), Quaternion(1.0))
print(pose2)

print(pose1*transformation_matrix)

tmp6 = PlanParameters('left_arm', tmp5)

tmp7 = EndEffectorLimits(1.0, 1.0, 1.0, 1.0)
print(tmp7)

tmp8 = JointStates(tmp2, tmp2, None)
print(tmp8)

pdb.set_trace()
