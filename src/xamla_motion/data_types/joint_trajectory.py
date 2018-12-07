# joint_trajectory.py
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

from datetime import timedelta
import numpy as np
import rospy
from std_msgs.msg import Header
import trajectory_msgs

from .joint_set import JointSet
from .joint_trajectory_point import JointTrajectoryPoint
from collections import Iterable

from typing import Callable


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
        if state:
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

    """
    Class JointTrajectory


    Methods
    -------
    empty()
        Creates a empty JointTrajectory instance
    to_joint_trajectory_msg(self, seq=0, frame_id='')
        Converts JointTrajectory to JointTrajectory ros message
    """

    def __init__(self, joint_set, points, valid=True):
        """
        Initialize class JointTrajectory

        Parameters
        ----------
        joint_set : JointSet
            Set of joints for which JointTrajectory should be defined
        points : Iterable[JointTrajectoryPoint]
            An Iterable of JointTrajectoryPoint which concretely defines
            the trajectory
        valid : bool (optinal)
            Defines if trajectory is avalid trajectory or not

        Returns
        -------
        JointTrajectory
            An instance of JointTrajectory

        Raises
        ------
        TypeError
            If joint_set is not of type JointSet or
            if points is not of type iterable of
            JointTrajectoryPoints or None
        ValueError
            If time from start is no ascending between points or
            joint set of points is not the same as of this JointTrajectory
        """

        if not isinstance(joint_set, JointSet):
            raise TypeError('joint_set is not of expected type JointSet')

        if (points != None and (not isinstance(points, Iterable) and
                                any(not isinstance(p, JointTrajectoryPoint)
                                    for p in points))):
            raise TypeError('points is not one of expected types None or '
                            'Iterable of JointTrajectoryPoints')

        self.__flags = JointTrajectoryFlags(14)

        if valid:
            self.__flags.is_valid = True

        self.__joint_set = joint_set

        if points is None:
            self.__points = None
            return
        else:
            self.__points = tuple(points)

        last_time = timedelta(days=0, seconds=0, microseconds=0)
        for i, p in enumerate(points):
            if p.time_from_start < last_time:
                raise ValueError('time_from_start values of trajectory'
                                 ' points must be acending.'
                                 ' Not the case for element: '+str(i))

            last_time = p.time_from_start

            if p.joint_set != self.__joint_set:
                raise ValueError('Provided trajectory point ' + str(i) +
                                 ' has values for different joint set')

            if p.velocities is None:
                self.__flags.has_velocity = False

            if p.accelerations is None:
                self.__flags.has_acceleration = False

            if p.efforts is None:
                self.__flags.has_effort = False

    @staticmethod
    def empty():
        """
        Creates a empty JointTrajectory instance
        """
        return JointTrajectory(JointSet.empty(), None)

    @property
    def joint_set(self):
        """
        joint_set : JointSet (readonly)
            Set of joints for which the trajectory contains values
        """
        return self.__joint_set

    @property
    def points(self):
        """
        points : List[JointTrajectoryPoints] (readonly)
            List of JointTrajectoryPoints which define the trajectory
        """
        return self.__points

    @property
    def is_valid(self):
        """
        is_valid : bool (readonly)
            True if trajectory is a valid trajectory
        """
        return self.__flags.is_valid

    @property
    def has_velocity(self):
        """
        has_velocity : bool (readonly)
            True if trajectory has velocities for every point
        """
        return self.__flags.has_velocity

    @property
    def has_acceleration(self):
        """
        is_acceleration : bool (readonly)
            True if trajectory has accelerations for every point
        """
        return self.__flags.has_acceleration

    @property
    def has_effort(self):
        """
        has_effort : bool (readonly)
            True if trajectory has efforts for every point
        """
        return self.__flags.has_effort

    @property
    def positions(self):
        """
        positions : List[JointValues]
            List of JointValues where each item is the positions field
            of JointTrajectoryPoints
        """
        return [p.positions for p in self.__points]

    @property
    def velocities(self):
        """
        velocities : List[JointValues]
            List of JointValues where each item is the velocities field
            of JointTrajectoryPoints
        """
        return [p.velocities for p in self.__points]

    @property
    def accelerations(self):
        """
        accelerations : List[JointValues]
            List of JointValues where each item is the accelerations field
            of JointTrajectoryPoints
        """
        return [p.accelerations for p in self.__points]

    @property
    def efforts(self):
        """
        efforts : List[JointValues]
            List of JointValues where each item is the efforts field
            of JointTrajectoryPoints
        """
        return [p.efforts for p in self.__points]

    @property
    def time_from_start(self):
        """
        timedelta : List[datatime.timedelta]
            List of timedeltas where each item is the time_from_start field
            of JointTrajectoryPoints
        """
        return [p.time_from_start for p in self.__points]

    def transform(self, transform_function: Callable[[JointTrajectoryPoint], JointTrajectoryPoint]):
        """
        Apply transformation to trajectory points

        Parameters
        ----------
        transform_function : Callable[[JointTrajectoryPoint], JointTrajectoryPoint]
            Function which defines the transformation which is applied on each
            joint trajectory point

        Returns
        -------
        JointTrajectory
            Joint Trajectory with transformed points
        """

        return type(self)(self.__joint_set, list(map(transform_function, self.__points)), self.is_valid)

    def to_joint_trajectory_msg(self, seq=0, frame_id=''):
        """
        Converts JointTrajectory to JointTrajectory ros message

        trajectory_msgs/JointTrajectory.msg

        Parameters
        ----------
        seq : int (default 0)
            set seq field of message header
        frame_id : int (default 0)
            set frame_id field of message header

        Returns
        -------
        JointTrajectory ros message
            Instance of JointTrajectory ros message
        """

        msg = trajectory_msgs.msg.JointTrajectory()

        msg.joint_names = self.__joint_set.names
        msg.points = [p.to_joint_trajectory_point_msg()
                      for p in self.__points]

        msg.header.seq = seq
        msg.header.frame_id = frame_id
        msg.header.stamp = rospy.Time.now()

        return msg

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
        return self.__points[key]

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

        if other.joint_set != self.__joint_set:
            return False

        for i, point in enumerate(other):
            if point != self.__points[i]:
                return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)
