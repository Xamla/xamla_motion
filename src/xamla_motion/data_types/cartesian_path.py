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

from .joint_set import JointSet
from .pose import Pose

import xamlamoveit_msgs.msg as xamla_msg
from typing import Iterable


class CartesianPath(object):
    """
    CartesianPath class describes a path by a list of joint configurations

    Methods
    -------
    from_one_point(point)
        Initialization of CartesianPath class with only one Point
    from_start_stop_point(start, stop)
        Initialization of CartesianPath class with start, stop point
    prepend(points)
        Creates new CartesianPath with points added in front of path points
    append(points)
        Creates new CartesianPath with points added behind the path points
    concat(other)
        Creates new CartesianPath with concatenated path points
    transform(transform_function)
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

        if points:
            frame_id = points[0].frame_id
            if any(p.frame_id != frame_id for p in points):
                raise ValueError('poses have not the same frame_id')

        self.__points = tuple(points)

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

    @classmethod
    def from_cartesian_path_msg(cls, msg):
        """
        Create instance from cartesian path message

        Parameters
        ----------
        msg : xamlamoveit_msgs.msg CartesianPath.msg
            ROS cartesian path message

        Returns
        -------
        CartesianPath
            New instance of CartesianPath
        """

        if msg.points:
            return cls([Pose.from_posestamped_msg(p) for p in msg.points])

    @property
    def points(self):
        """
        points : Deque[Pose] (readonly)
            A deque of Pose which represent the cartesian poses
            which are describe the path
        """
        return self.__points

    @property
    def positions(self):
        """
        positions : List[np.array((3,),dtype=floating)
            positions in x,y,z coordinates
        """
        return [p.translation for p in self.__points]

    @property
    def orientations(self):
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

        if isinstance(points, Pose):
            return type(self)((points,)+self.__points)

        if (not isinstance(points, Iterable) or
            any(not isinstance(j, Pose)
                for j in points)):
            raise TypeError('points is not of one of expected types'
                            ' Pose or Iterable[Pose]')

        return type(self)(tuple(points)+self.__points)

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

        if isinstance(points, Pose):
            return type(self)(self.__points+(points,))

        if (not isinstance(points, Iterable) or
                any(not isinstance(j, Pose)
                    for j in points)):
            raise TypeError('points is not of one of expected types'
                            ' Pose or Iterable[Pose]')

        return type(self)(self.__points+tuple(points))

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

        return type(self)(self.__points+other.points)

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
            return self.__class__([transform_function(x)
                                   for x in self.__points])
        except TypeError as exc:
            raise TypeError('None valid transformation function') from exc

    def to_cartesian_path_msg(self):
        """
        Generates a xamlamoveit_msgs.msg CartesianPath.msg from this CartesianPath instance
        """
        path = xamla_msg.CartesianPath()
        path.points = [p.to_posestamped_msg() for p in self.__points]

        return path

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

        for i, point in enumerate(other):
            if point != self.__points[i]:
                return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)
