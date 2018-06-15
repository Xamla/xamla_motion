#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from future.builtins import *
from JointSet import JointSet
from JointValues import JointValues
import pdb
import copy

tmp0 = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
values = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
print(type(tmp0))
tmp1 = JointSet(tmp0)

tmp2 = JointValues(tmp1, values)

tmp3 = tmp2.reorder(
    JointSet(['Joint2', 'Joint1', 'Joint4', 'Joint3', 'Joint6', 'Joint5']))

tmp4 = JointValues(JointSet(['Joint2', 'Joint5', 'Joint1']), values[1:-2])

print(4/tmp2)
print((1, 2, 3, 4, 10, 6)/tmp2)
print(tmp2/4)
print(tmp2/(1, 2, 3, 10, 5, 1))
print(tmp2/tmp4)
print(tmp4/tmp2)

pdb.set_trace()
