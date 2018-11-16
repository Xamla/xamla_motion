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

#!/usr/bin/env python3

from .joint_set import JointSet
from .joint_values import JointValues

from typing import Iterable


class JointPath(object):
    """
    JointPath class describes a path by a list of joint configurations

    Methods
    -------
    from_one_point(point)
        Initialization of JointPath class with only one Point
    from_start_stop_point(start, stop)
        Initialization of JointPath class with start, stop point
    prepend(points)
        Creates new JointPath with points added in front of path points
    append(points)
        Creates new JointPath with points added behind the path points
    concat(other)
        Creates new JointPath with concatenated path points
    transform(transform_function)
        Creates a transformed version of JointPath
    """

    def __init__(self, joints, points):
        """
        Initialization of JointPath class

        Parameters
        ----------
        joints : JointSet
            Set of joints
        points : Iterable[JointValues]
             Iterable of JointValues where each JointValues instance
             describes a point of the path

        Returns
        -------
        JointPath
            Instance of JointPath

        Raises
        ------
        TypeError : type mismatch
            If joints is not of type JointSet
            or if points is not of type list of JointValues
        """

        if not isinstance(joints, JointSet):
            raise TypeError('joints is not of expected type JointSet')

        if (not isinstance(points, Iterable) or
                any(not isinstance(j, JointValues)
                    for j in points)):
            raise TypeError('points is not of expected'
                            ' type Iterable of JointValues')

        self.__joints = joints
        self.__points = tuple(self._align_values(j) for j in points)

    def _align_values(self, joint_values):
        if joint_values.joint_set == self.__joints:
            return joint_values

        if not joint_values.joint_set.is_similar(self.__joints):
            raise ValueError('Provided path points have joint values'
                             ' of incompatible joint sets')

        return joint_values.reorder(self.__joints)

    @classmethod
    def from_one_point(cls, point):
        """
        Initialization of JointPath class with only one Point

        Parameters
        ----------
        point : JointValues
             Single point to initialize JointPath

        Returns
        -------
        JointPath
            Instance of JointPath

        Raises
        ------
        TypeError : type mismatch
            If points is not of type JointValues
        """

        if not isinstance(point, JointValues):
            raise TypeError('joint_values is not of expected'
                            ' type JointValues')

        return cls(point.joint_set, [point])

    @classmethod
    def from_start_stop_point(cls, start, stop):
        """
        Initialization of JointPath class with start, stop point

        Parameters
        ----------
        start : JointValues
             start point of the joint path
        stop : JointValues
             stop point of the joint path

        Returns
        -------
        JointPath
            Instance of JointPath

        Raises
        ------
        TypeError : type mismatch
            If start or stop is not of type JointValues
        """

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
        return self.__points

    def prepend(self, points):
        """
        Creates new JointPath with points added in front of path points

        Parameters
        ----------
        points : JointValues or Iterable[JointValues]
            Points which are added in front of path points

        Returns
        -------
        JointPath
            Instance of JointPath with added points

        Raises
        ------
        TypeError
            If points is not one of expected types
            JointValues or Iterable of JointValues
        """

        if isinstance(points, JointValues):
            return self.__class__(self.__joints, (points,) + self.__points)

        if (not isinstance(points, Iterable) or
                any(not isinstance(j, JointValues)
                    for j in points)):
            raise TypeError('points is not of expected'
                            ' type JointValues or Iterable of JointValues')

        return self.__class__(self.__joints, (points) + self.__points)

    def append(self, points):
        """
        Creates new JointPath with points added behind the path points

        Parameters
        ----------
        points : JointValues or Iterable[JointValues]
            Points which are added behind path points

        Returns
        -------
        JointPath
            Instance of JointPath with added points

        Raises
        ------
        TypeError
            If points is not one of expected types
            JointValues or Iterable of JointValues
        """

        if isinstance(points, JointValues):
            return self.__class__(self.__joints, self.__points + (points,))

        if (not isinstance(points, Iterable) or
                any(not isinstance(j, JointValues)
                    for j in points)):
            raise TypeError('points is not of expected'
                            ' type JointValues or Iterable of JointValues')

        return self.__class__(self.__joints, self.__points + (points))

    def concat(self, other):
        """
        Creates new JointPath with concatenated path points

        Parameters
        ----------
        other : JointPath

        Raises
        ------
        TypeError
            If other is not of type JointPath
        """
        if not isinstance(other, JointPath):
            raise TypeError('other is not of expected type JointPath')

        return self.__class__(self.__joints, self.__points+other.points)

    def transform(self, transform_function):
        """
        Creates a transformed version of JointPath

        The transformation which is applied to every point in
        JointPath is defined by the transform function

        Parameters
        ----------
        transform_function : callable or numpy.ufunc
            Function which is applied to every point value

        Returns
        ------
        JointPath
            A new Instance of JointPath with transformed
            point values

        Raises
        ------
        TypeError : type mismatch
            If transform function is not callable or not
            a numpy.ufunc and if the function dont has the
            signature input : floating , output : floating
        """

        try:
            return self.__class__(self.__joints,
                                  [x.transform(transform_function)
                                   for x in self.__points])
        except TypeError as exc:
            raise TypeError('None valid transformation function') from exc

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
            except IndexError:
                raise IndexError('index out of range')
        else:
            raise TypeError(
                'key is not one of expected types int or slice ')

    def __len__(self):
        return len(self.__points)

    def __iter__(self):
        return iter(self.__points)

    def __str__(self):
        return str(self.__points)

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False

        if id(other) == id(self):
            return True

        if other.joint_set != self.__joints:
            return False

        for i, point in enumerate(other):
            if point != self.__points[i]:
                return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)
