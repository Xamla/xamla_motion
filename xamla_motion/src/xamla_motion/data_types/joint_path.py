# joint_path.py
#
# Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function)  # , unicode_literals)
# from future.builtins import *
from future.utils import raise_from, raise_with_traceback

from data_types import JointSet
from data_types import JointValues

from collections import Iterable, deque


class JointPath(object):
    """
    JointPath class describes a path by a list of joint configurations

    Methods
    -------
    """

    def __init__(self, joints, points):

        if not isinstance(joints, JointSet):
            raise TypeError('joints is not of expected type JointSet')

        if (not isinstance(points, collections.Iterable) or
                any(not isinstance(j, JointValues)
                    for j in points)):
            raise TypeError('points is not of expected'
                            ' type Iterable of JointValues')

        self.__joints = joints
        self.__points = deque(self._align_values(j) for j in points)

    def _align_values(self, joint_values):
        if joint_values.joint_set == self.__joints:
            return joint_values

        if not joint_values.is_similar(self.__joints):
            raise ValueError('Provided path points have joint values'
                             ' of incompatible joint sets')

        return joint_values.reorder(self.__joints)

    @classmethod
    def from_one_point(cls, point):
        if not isinstance(point, JointValues):
            raise TypeError('joint_values is not of expected'
                            ' type JointValues')

        return cls(point.joint_set, [point])

    @classmethod
    def from_start_stop_point(cls, start, stop):
        if (not isinstance(start, JointValues) or
                not isinstance(stop, JointValues)):
            raise TypeError('start or/and stop are not of'
                            'expected type JointValues')

        return cls(start.joint_set, [start, stop])

    @property
    def joint_set(self):
        """
        joint_set : JointSet (readonly)
            JointSet which represent the joints for which JointPath
            describes a path
        """
        return self.__joints

    @property
    def points(self):
        """
        points : Deque[JointValues] (readonly)
            A deque of JointValues which represent the joint configurations
            which are describe the path
        """
        return self.__joints

    def prepend(self, points):

        if isinstance(points, JointValues)
            return self.__class__(self.__joints, points.extendleft(points))

        return self.__class__(self.__joints,
                              points.extendleft(reversed(points)))

    def append(self, points):

        if isinstance(points, JointValues)
            return self.__class__(self.__joints, points.extend(points))

        return self.__class__(self.__joints, points.extend(points))

    def concat(self, other):
        return self.__class__(self.__joints, points.extend(other.points))

    def sub(self, start_index, end_index):
        return self.__class__(self.__joints,
                              self.__points[start_index:end_index])

    def transform(self, transform_function):
        return self.__class__(self.__joints,
                              [x.transform(transform_function)
                               for x in self.__points])

    def __getitem__(self, key):
        """
        Returns value by joint name or index

        Parameters
        ----------
        key : int  or slice
            index of joint or slice for which the values are requested

        Returns
        -------
        JointValues or List[JointValues]
            Returns a instance of JointValues or list of JointValues 

        Raises
        ------
        TypeError : type mismatch
            If key is not int or slice
        IndexError :
            If index is out of range

        """
        if isinstance(key, (int, slice)):
            try:
                return self.__points[key]
            except IndexError as exc:
                raise IndexError('index out of range')
        else:
            raise TypeError(
                'key is not one of expected types int, str or slice ')

    def __len__(self):
        return len(self.__points)

    def __iter__(self):
        return self.__points.__iter__()

    def __str__(self):
        return self.__points.__str__()

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False

        if id(other) == id(self):
            return True

        if other.joints != self.__joints:
            return False

        for i, point in enumerate(other):
            if point != self.__points[i]:
                return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)
