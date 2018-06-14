#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from future.builtins import *
from JointSet import JointSet
from JointValues import JointValues
import pdb

tmp0 = ['Joint1', 'Joint2']
values = [1.0, 2.0]
print(type(tmp0))
tmp1 = JointSet(tmp0)

tmp2 = JointValues(tmp1, values)

tmp3 = tmp2.reorder(JointSet(['Joint2', 'Joint1']))
pdb.set_trace()
