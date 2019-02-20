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

import bisect
from collections import Iterable
from datetime import timedelta
from typing import Callable, Union

import numpy as np
import rospy
import trajectory_msgs
from std_msgs.msg import Header

from .joint_set import JointSet
from .joint_trajectory_point import JointTrajectoryPoint


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

    @property
    def duration(self):
        """
        duration : timedelta
            duration of trajectory
        """

        return self.__points[-1].time_from_start

    def append(self, other: 'JointTrajectory', delay: timedelta=timedelta(0)):
        """
        Append a JointTrajectory to current trajectory

        Parameters
        ----------
        other : JointTrajectory
            joint trajectory to append
        delay : timedelta
            time between last point of current
            trajectory and first point of other trajectory

        Returns
        -------
        JointTrajectory
        """

        duration = self.duration + delay

        return type(self)(self.__joint_set,
                          self.__points+(p.add_time_offset(duration) for p in other.points))

    def prepend(self, other: 'JointTrajectory', delay: timedelta=timedelta(0)):
        """
        Prepend a JointTrajectory to current trajectory

        Parameters
        ----------
        other : JointTrajectory
            joint trajectory to prepend
        delay : timedelta
            time between last point of other
            trajectory and first point of current trajectory

        Returns
        -------
        JointTrajectory
        """

        duration = other.duration + delay

        return type(self)(other.joint_set,
                          other.points+(p.add_time_offset(duration) for p in self.__points))

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

    def get_point_before(self, time: timedelta, return_index: bool=False):
        """
        Find and return JointTrajectoryPoint nearest and before defined time

        Parameters
        ----------
        time : timedelta
            defines time for which the nearest and before point is required
        return_index: bool (default False)
            If True return index instead of JointTrajectoryPoint

        Returns
        -------
        JointTrajectoryPoint or int
            If return_index True returns index instead of JointTrajectoryPoint 
        """

        idx = bisect.bisect_left(self.time_from_start, time)-1

        if idx < 0:
            idx = 0

        if return_index:
            return idx

        return self.__points[idx]

    def evaluate_at(self, time: timedelta):
        """
        Evaluates the trajectory at a given time

        Parameters
        ----------
        time : timedetla
            The time at which the trajectory should be evaluated
        """
        start_t = self.__points[0].time_from_start
        end_t = self.__points[-1].time_from_start

        if time < start_t or time > end_t:
            raise ValueError('time: {} is out of bounds (min: {}, '
                             'max: {}'.format(time, start_t, end_t))
        idx = self.get_point_before(time, return_index=True)

        next_idx = min(idx+1, len(self))

        return self.__points[idx].interpolate_cubic(self.__points[next_idx], time)

    def merge(self, other: 'JointTrajectory', delay_self: Union[timedelta, None]=None,
              delay_other: Union[timedelta, None]=None):
        """
        Merge self and other joint trajectory

        Parameters
        ----------
        other: JointTrajectory
            Other trajectory to merge
        delay_self: Union[timedelta,None] (default=None)
            If defined delay which is applied to self trajectory
        delay_other: Union[timedelta,None] (default=None)
            If defined delay which is applied to other trajectory

        Returns
        -------
        JointTrajectory
            Merged joint trajectory
        """

        union_joint_set = self.joint_set.union(other.joint_set)

        num_points_self = len(self)
        num_points_other = len(other)

        duration_self = self.__points[-1].time_from_start
        duration_other = other.points[-1].time_from_start

        if delay_self is not None:
            duration_self += delay_self

        if delay_other is not None:
            duration_other += delay_other

        if duration_self > duration_other:
            duration = duration_self
        else:
            duration = duration_other

        max_points = max(num_points_self, num_points_other)

        dt = duration / (max_points-1)
        target_time = [dt*i for i in range(max_points)]

        merged_points = []


        for t in target_time:
            if delay_self is not None:
                time_s = t-delay_self
            else:
                time_s = t
            # Guard to clamp time to allowed intervall
            start_t = self.time_from_start[0]
            end_t = self.time_from_start[-1]
            time_s = max(time_s, start_t)
            time_s = min(time_s, end_t)
            p_s = self.evaluate_at(time_s).with_time_from_start(t)

            if delay_other is not None:
                time_o = t-delay_other
            else:
                time_o = t
            # Guard to clamp time to allowed intervall
            start_t = other.time_from_start[0]
            end_t = other.time_from_start[-1]
            time_o = max(time_o, start_t)
            time_o = min(time_o, end_t)
            p_o = other.evaluate_at(time_o).with_time_from_start(t)

            merged_points.append(p_s.merge(p_o))

        return type(self)(union_joint_set, merged_points)

    def to_joint_trajectory_msg(self, seq=0, frame_id=''):
        """
        Converts JointTrajectory to JointTrajectory ros message

        trajectory_msgs/JointTrajectory.msg

        Parameters
        ----------
        seq: int(default 0)
            set seq field of message header
        frame_id: int(default 0)
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
        key: int or slice
            index of joint or slice for which the values are requested

        Returns
        -------
        JointValues or List[JointValues]
            Returns a instance of JointValues or list of JointValues

        Raises
        ------
        TypeError: type mismatch
            If key is not int or slice
        IndexError:
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
