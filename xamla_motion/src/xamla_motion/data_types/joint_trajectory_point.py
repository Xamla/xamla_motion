# joint_trajectory_points.py
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
from future.builtins import map
from future.utils import raise_from, raise_with_traceback

from datetime import timedelta
import numpy as np

from data_types import JointSet, JointValues
from trajectory_msgs.msg import JointTrajectoryPoint


class JointTrajectoryPoint(object):
    """
    Class JointTrajectoryPoint

    Methods
    -------
    from_joint_trajectory_point_msg
        Creates an instance of JointTrajectoryPoint from ros message
    add_time_offset
        Creates a new instance of this JointTrajectoryPoint with same
        position, vel, acc, eff but with a modified time_from_start
    """

    def __init__(self, time_from_start, positions, velocities=None,
                 accelertations=None, efforts=None):
        """ 
        Initialize JointTrajectoryPoint class

        Parameters
        ----------
        time_from_start : datatime.timedelta
            duration between start of trajectory and this point 
        positions : JointValues
            joint positions
        velocities : JointValues or None
            joint velocities
        accelerations : JointValues
            joint accelerations
        efforts : JointValues
            joint force for a prismatic joint or torque 
            for a rotational joint

        Returns
        -------
        JointTrajectoryPoint
            Instance of JointTrajectoryPoint

        Raises
        ------
        TypeError : type mismatch
            If time_from_start is not of expected type datatime.timedelta
            or if positions is not of expected type JointValues
            or if velocities, accelerations or efforts ist not one
            of the expected types None or JointValues
        ValueError
            If the joint set of positions and if not None 
            velocities, accelerations and efforts is not equal
        """

        if not isinstance(time_from_start, timedelta):
            raise TypeError('time_from_start is not of'
                            ' expected type timedelta')

        self.__time_from_start = time_from_start

        self.__positions = self._init_check(positions, 'positions', False)
        self.__velocities = self._init_check(velocities, 'velocities')
        self.__accelerations = self._init_check(accelertations,
                                                'accelerations')
        self.__efforts = self._init_check(efforts, 'efforts')

    def _init_check(self, joint_values, name, check=True):
        if not joint_values:
            return None

        if not isinstance(joint_values, JointValues):
            raise TypeError(name+' is not of expected type JointValues')

        if check == True:
            if joint_values.joint_set != self.joint_set:
                raise ValueError('joint names or order do not match.')

        return joint_values

    @classmethod
    def from_joint_trajectory_point_msg(cls, joint_set, msg):
        """
        Creates an instance of JointTrajectoryPoint from ros message

        Creates an instance of JointTrajectoryPoint from the ros message
        trajectory_msgs/JointTrajecoryPoint

        Parameters
        ----------
        joint_set : JointSet
            Set of joints for which the trajectroy point is defined
        msg : trajectory_msgs/JointTrajectoryPoint
            Instance of ros message that should be converted to 
            JointTrajectoryPoint

        Returns
        -------
        JointTrajectoryPoint
            New instance of JointTrajectoryPoint with the values 
            of the ros message

        Raises
        ------
        TypeError : type mismatch
            If values from msg are not convertable to numpy
            array of type floating or if joint_set is not of
            expected type JointSet
        """

        if not isinstance(joint_set, JointSet):
            raise TypeError('joint_set is not of expected type JointSet')

        j_positions = JointValues(joint_set, msg.positions)
        j_velocity = cls._init_from_msg_helper(joint_set, msg.velocities)
        j_accelerations = cls._init_from_msg_helper(joint_set,
                                                    msg.accelerations)
        j_effort = cls._init_from_msg_helper(joint_set, msg.effort)

        days = msg.time_from_start.secs // (24*3600)
        seconds = msg.time_from_start.secs % (24*3600)
        microseconds = msg.time_from_start.nsecs // 1000

        time_from_start = timedelta(days=days, seconds=seconds,
                                    microseconds=microseconds)

        return cls(time_from_start, j_positions, j_velocity,
                   j_accelerations, j_effort)

    @staticmethod
    def _init_from_msg_helper(joint_set, values):
        if values:
            return JointValues(joint_set, values)
        else:
            return None

    @property
    def joint_set(self):
        """
        joint_set : JointSet
            Set of joints for which the trajectory point contains values
        """
        return self.__positions.joint_set

    @property
    def time_from_start(self):
        """
        time_from_start : datetime.timedelta (readonly)
            Time span between trajectory start and this trajectory 
            point as a instance of datetime.timedelta
        """
        return self.__time_from_start

    @property
    def positions(self):
        """
        positions : JointValues (readonly)
            Joint positions of this trajectory point
        """
        return self.__positions

    @property
    def velocities(self):
        """
        velocities : JointValues (readonly)
            If defined available joint velocites of this trajectory point
            else None
        """
        return self.__velocities

    @property
    def accelerations(self):
        """
        accelerations : JointValues (readonly)
            If defined available joint accelerations of this trajectory point
            else None
        """
        return self.__accelerations

    @property
    def efforts(self):
        """
        efforts : JointValues (readonly)
            If defined available joint efforts of this trajectory point
            else None
        """
        return self.__efforts

    def add_time_offset(self, offset):
        """
        Creates a new instance of JointTrajectoryPoint with applied time offset

        adds a time offset to time_from_start

        Parameters
        ----------
        offset : datatime.timedelta
            time offset as datatime timedelta

        Returns
        -------
            Instance of JointTrajectoryPoint with modified time_from_start

        Raises
        ------
        TypeError
            If offset ist not of expected type datatime.timedelta
        """
        if not isinstance(offset, timedelta):
            raise TypeError('offset is not of expected'
                            ' type datatime.timedelta')

        time_from_start = self.__time_from_start + offset

        return self.__clas__(time_from_start,
                             self.__positions,
                             self.__velocities,
                             self.__accelerations,
                             self.__efforts)

    def __str__(self):
        s = '\n'.join(['  '+k+' = ' + str(v)
                       for k, v in self.__dict__.items()])
        s = s.replace('_'+self.__class__.__name__+'__', '')
        return self.__class__.__name__+'\n'+s

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False

        if id(other) == id(self):
            return True

        if other.time_from_start != self.__time_from_start:
            return False

        if other.positions != self.__positions:
            return False

        if other.velocites != self.__velocities:
            return False

        if other.accelerations != self.__accelerations:
            return False

        if other.efforts != self.__efforts:
            return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)
