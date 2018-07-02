from __future__ import (absolute_import, division,
                        print_function)  # , unicode_literals)
from future.builtins import map
from future.utils import raise_from, raise_with_traceback

from datetime import timedelta
import numpy as np

from data_types import JointSet
from data_types import JointTrajectoryPoint
from collections import Iterable


class JointTrajectoryFlags(object):
    __is_valid = 1 << 0
    __has_velocity = 1 << 1
    __has_acceleration = 1 << 2
    __has_effort = 1 << 3

    def __init__(self, Type):
        self.__value = Type

    @property
    def is_valid(self):
        return self.__value & self.__is_valid > 0

    @is_valid.setter
    def is_valid(self, state):
        if state:
            self.__value |= self.__is_valid
        else:
            self.__value &= (~self.__is_valid)

    @property
    def has_velocity(self):
        return self.__value & self.__has_velocity > 0

    @has_velocity.setter
    def has_velocity(self, state):
        if state:
            self.__value |= self.__has_velocity
        else:
            self.__value &= (~self.__has_velocity)

    @property
    def has_acceleration(self):
        return self.__value & self.__has_acceleration > 0

    @has_acceleration.setter
    def has_acceleration(self, state):
        if state:
            self.__value |= self.__has_acceleration
        else:
            self.__value &= (~self.__has_acceleration)

    @property
    def has_effort(self):
        return self.__value & self.__has_effort > 0

    @has_effort.setter
    def has_effort(self, state):
        if has_effort:
            self.__value |= self.__has_effort
        else:
            self.__value &= (~self.__has_effort)

    def __str__(self):
        l = []

        if self.__value & JointTrajectoryFlags.__is_valid > 0:
            l.append('is_valid')
        if self.__value & JointTrajectoryFlags.__has_velocity > 0:
            l.append('has_velocity')
        if self.__value & JointTrajectoryFlags.has_acceleration > 0:
            l.append('has_acceleration')
        if self.__value & JointTrajectoryFlags.has_effort > 0:
            l.append('has_effort')

        return ', '.join(l)

    def __eq__(self, y):
        return self.__value == y.__value

    def __or__(self, other):
        return self.__class__(self.__value | other)

    def __ror__(self, other):
        return self.__or__(other)

    def __and__(self, other):
        return self.__class__(self.__value & other)

    def __rand__(self, other):
        return self.__and__(other)


class JointTrajectory(object):

    def __init__(self, joint_set, points, valid=True):

        if not isinstance(joint_set, JointSet):
            raise TypeError('joint_set is not of expected type JointSet')

        if (points != None and (not isinstance(points, Iterable) and
                                any(not isinstance(p, JointTrajectoryPoint)
                                    for p in points))):
            raise TypeError('points is not one of expected type None or '
                            'Iterable of JointTrajectoryPoints')

        self.__flags = JointTrajectoryFlags(0)

        if valid:
            self.__flags.is_valid = True

        self.__joint_set = joint_set

        if points == None:
            self.__points = None
            return

        last_time = timedelta(days=0, seconds=0, mircoseconds=0)
        for i, p in enumerate(points):
            if p.time_from_start < last_time:
                pass

    @staticmethod
    def empty():
        return JointTrajectory(JointSet.empty(), None)
