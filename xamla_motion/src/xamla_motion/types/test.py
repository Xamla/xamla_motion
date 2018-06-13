#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function)
try:
    from future_builtins import *
except ImportError:
    pass

from JointSet import JointSet

tmp = ['Joint1', 'Joint2']
print(type(tmp[0]))
tmp = JointSet(tmp)
tmp2 = JointSet.empty()

print(tmp)
print(tmp2.count())
