# cartesian_path.py
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

from data_types import JointSet
from data_types import Pose

from collections import Iterable, deque


class CartesianPath(object):
    """
    CartesianPath class describes a path by a list of joint configurations

    Methods
    -------
    from_one_point
        Initialization of CartesianPath class with only one Point
    from_start_stop_point
        Initialization of CartesianPath class with start, stop point
    prepend
        Creates new CartesianPath with points added in front of path points
    append
        Creates new CartesianPath with points added behind the path points
    concat
        Creates new CartesianPath with concatenated path points
    transform
        Creates a transformed version of CartesianPath
    """

    def __init__(self, points):
        """
        Initialization of CartesianPath class

        Parameters
        ----------
        points : Iterable[Pose]
             List of Pose where each Pose instance
             describes a point of the path

        Returns
        -------
        CartesianPath
            Instance of CartesianPath

        Raises
        ------
        TypeError : type mismatch
            If points is not of type iterable of Pose
        """

        if (not isinstance(points, Iterable) or
                any(not isinstance(j, Pose)
                    for j in points)):
            raise TypeError('points is not of expected'
                            ' type Iterable of Pose')

        self.__points = deque(points)

    @classmethod
    def from_one_point(cls, point):
        """
        Initialization of CartesianPath class with only one Point

        Parameters
        ----------
        point : Pose
             Single point to initialize CartesianPath

        Returns
        -------
        CartesianPath
            Instance of CartesianPath

        Raises
        ------
        TypeError : type mismatch
            If points is not of type Pose
        """

        if not isinstance(point, Pose):
            raise TypeError('joint_values is not of expected'
                            ' type Pose')

        return cls([point])

    @classmethod
    def from_start_stop_point(cls, start, stop):
        """
        Initialization of CartesianPath class with start, stop point

        Parameters
        ----------
        start : Pose
             start point of the joint path
        stop : Pose
             stop point of the joint path

        Returns
        -------
        CartesianPath
            Instance of CartesianPath

        Raises
        ------
        TypeError : type mismatch
            If start or stop is not of type Pose
        """

        if (not isinstance(start, Pose) or
                not isinstance(stop, Pose)):
            raise TypeError('start or/and stop are not of'
                            'expected type Pose')

        return cls([start, stop])

    @property
    def points(self):
        """
        points : Deque[Pose] (readonly)
            A deque of Pose which represent the cartesian poses
            which are describe the path
        """
        return self.__points

    @property
    def Positions(self):
        """
        positions : List[np.array((3,),dtype=floating)
            positions in x,y,z coordinates
        """
        return [p.translation for p in self.__points]

    def Orientations(self):
        """
        orientations : List[pyquaternion.Quaternion]
            orientations as Quaternion from pyquaternion
        """
        return [o.quaternion for o in self.__points]

    def prepend(self, points):
        """
        Creates new CartesianPath with points added in front of path points

        Parameters
        ----------
        points : Pose or Iterable[Pose]
            Points which are added in front of path points

        Returns
        -------
        CartesianPath
            Instance of CartesianPath with added points

        Raises
        ------
        TypeError
            If points is not one of expected types
            Pose or Iterable of Pose
        """

        new_points = deepcopy(self.__points)
        if isinstance(points, Pose):
            new_points.appendleft(points)
            return self.__class__(self.__joints, new_points)

        if (not isinstance(points, collections.Iterable) or
                any(not isinstance(j, Pose)
                    for j in points)):
            raise TypeError('points is not of expected'
                            ' type Pose or Iterable of Pose')

        new_points.extendleft(list(reversed(points)))
        return self.__class__(self.__joints, new_points)

    def append(self, points):
        """
        Creates new CartesianPath with points added behind the path points

        Parameters
        ----------
        points : Pose or Iterable[Pose]
            Points which are added behind path points

        Returns
        -------
        CartesianPath
            Instance of CartesianPath with added points

        Raises
        ------
        TypeError
            If points is not one of expected types
            Pose or Iterable of Pose
        """

        new_points = deepcopy(self.__points)
        if isinstance(points, Pose):
            new_points.append(points)
            return self.__class__(self.__joints, new_points)

        if (not isinstance(points, collections.Iterable) or
                any(not isinstance(j, Pose)
                    for j in points)):
            raise TypeError('points is not of expected'
                            ' type Pose or Iterable of Pose')

        new_points.extend(points)
        return self.__class__(self.__joints, new_points)

    def concat(self, other):
        """
        Creates new CartesianPath with concatenated path points

        Parameters
        ----------
        other : CartesianPath

        Raises
        ------
        TypeError
            If other is not of type CartesianPath
        """
        if not isinstance(other, CartesianPath):
            raise TypeError('other is not of expected type CartesianPath')

        new_points = deepcopy(self.__points)
        new_points.extend(points)
        return self.__class__(self.__joints, new_points)

    def transform(self, transform_function):
        """
        Creates a transformed version of CartesianPath

        The transformation which is applied to every point in
        CartesianPath is defined by the transform function

        Parameters
        ----------
        transform_function : callable
            Function which is applied to every pose

        Returns
        ------
        CartesianPath
            A new Instance of CartesianPath with transformed
            poses

        Raises
        ------
        TypeError : type mismatch
            If transform function is not callable and
            if the function dont has the
            signature input : Pose , output : Pose
        """

        try:
            return self.__class__(self.__joints,
                                  [transform_function(x)
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
        Pose or List[Pose]
            Returns a instance of Pose or list of Pose

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
                'key is not one of expected types int or slice ')

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

        for i, point in enumerate(other):
            if point != self.__points[i]:
                return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)
