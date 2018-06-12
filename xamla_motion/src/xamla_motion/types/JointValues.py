#!/usr/bin/env python

from __future__ import print_function, division, unicode_literals, absolute_import
from builtins import int, map
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
            self._values = values
        else:
            raise TypeError('values is not of expected'
                            ' types float or list[float]')
