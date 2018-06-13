#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function)
try:
    from future_builtins import *
except ImportError:
    pass
from functools import total_ordering

from . import JointSet


@total_ordering
class JointValues(object):

    def __init__(self, joint_set, values):
        self._joint_set = list()
        self._values = list()

        if not isinstance(join_set, JointSet):
            raise TypeError('joint_set is not expected type JointSet')

        self._joint_set = join_set

        if isinstance(values, float):
            self._values = [values] * joint_set.count()
        elif values and all(isinstance(value, float) for value in values):
            if and len(values) != self._joint_set.count():
                raise ValueError('values has not the same size'
                                 ' as JointSet')
            self._values = values
        else:
            raise TypeError('values is not of expected'
                            ' types float, list[float] or'
                            ' not of same size as JointSet')

    @staticmethod
    def empty():
        joint_values = JointValues(JointSet.empty(), 0.0)
        joint_values._values = []
        return joint_values

    @staticmethod
    def zero(joint_set):
        if not isinstance(joint_set, JointSet):
            raise TypeError('zero: joint_set is not expected type JointSet')
        joint_values = JointValues(joint_set, 0.0)
        return joint_values
