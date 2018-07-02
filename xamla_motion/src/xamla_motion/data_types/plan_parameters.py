# plan_parameters.py
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
#from future.builtins import *
from future.utils import raise_from, raise_with_traceback
from copy import deepcopy

from data_types import JointLimits
from xamla_motion_exceptions import ArgumentError

import numpy as np


class PlanParameters(object):
    """
    PlanParameter holds all constrains for joint space trajectory planning

    Methods
    -------
    from_arguments
        Creates instance of PlanParameters from
        limit numpy arrays
    """

    def __init__(self, move_group_name, joint_limits, **kwargs):
        """
        Initialization of PlanParameters class

        Parameters
        ----------
        move_group_name : str convertable
            name of the move group for which the plan parameters are created
        joint_limits : JointLimits
            Defines the joint limits of the move group
        argv : dict
            The argv dict is used to set parameters which have default values
            this are sample_resolution (default = 0.008),
            collision_check (default = True), max_deviation (default = 0.2),
            scale_velocity (default = 1.0) and scale_acceleration (
                default = 1.0)

        Returns
        ------
            Instance of PlanParameters

        Raises
        ------
            TypeError : type mismatch
                If joint_limits is not of expected type JointLimts 

        Examples
        --------
        >>> joints = ['Joint1','Joint2']
        >>> joint_set = JointSet(joints)
        >>> max_v = np.array([1.0, 1.5])
        >>> max_a = np.array([0.2, 0.4])
        >>> min_p = np.array([10, None])
        >>> max_p = np.array([None, None])
        >>> joint_limits = JointLimits(joint_set, max_v, max_a, min_p, max_p)
        >>> p = PlanParameters('left_arm', joint_Limits, scale_velocity=0.5)
        """

        self.__sample_resolution = kwargs.get("sample_resolution", 0.08)
        if not isinstance(self.__sample_resolution, float):
            raise TypeError('sample_resolution is not of expected type float')
        self.__collision_check = kwargs.get("collision_check", True)
        if not isinstance(self.__collision_check, bool):
            raise TypeError('collision_check is not of expected type bool')
        self.__max_deviation = kwargs.get("max_deviation", 0.2)
        if not isinstance(self.__sample_resolution, float):
            raise TypeError('sample_resolution is not of expected type float')
        self.__scale_velocity = kwargs.get("scale_velocity", 1.0)
        if not isinstance(self.__scale_velocity, float):
            raise TypeError('scale_velocity is not of expected type float')
        if self.__scale_velocity < 0.0 or self.__scale_velocity > 1.0:
            raise ValueError('scale_velocity is not in expected range'
                             'between 0.0 and 1.0')
        self.__scale_acceleration = kwargs.get("acceleration_velocity", 1.0)
        if not isinstance(self.__scale_acceleration, float):
            raise TypeError('scale_acceleration is not of expected type float')
        if self.__scale_acceleration < 0.0 or self.__scale_acceleration > 1.0:
            raise ValueError('scale_acceleration is not in expected range'
                             'between 0.0 and 1.0')

        if isinstance(joint_limits, JointLimits):
            max_s_velocity = joint_limits.max_velocity * self.__scale_velocity
            max_s_acceleration = (joint_limits.max_acceleration
                                  * self.__scale_acceleration)
            self.__joint_limits = JointLimits(joint_limits.joint_set,
                                              max_s_velocity,
                                              max_s_acceleration,
                                              joint_limits.min_position,
                                              joint_limits.max_position)
        else:
            raise TypeError('joint_limits is not'
                            ' of expected type JointLimits')

        if isinstance(move_group_name, str):
            self.__move_group_name = move_group_name
        else:
            raise TypeError('argument 1 (move_group_name) is not '
                            'of expected type str')

    @classmethod
    def from_arguments(cls, move_group_name, joint_set, max_velocity,
                       max_acceleration, min_position, max_position, **kwargs):
        """
        Initialization of PlanParameters class limit arrays

        Parameters
        ----------
        move_group_name : str convertable
            name of the move group for which the plan parameters are created
        joint_set : JointSet
            Set of joints for which joint limits are required
        max_velocity : list of float or numpy.array(dtype=floating)
            One dimension array which defines
            the maximal velocity for each joint
        max_acceleration : list of float or numpy.array(dtype=floating)
            One dimension array which defines
            the maximal acceleration for each joint
        min_position : list of float or numpy.array(dtype=floating)
            One dimension array which defines
            the mininmal position for each joint
        max_position : list of float or numpy.array(dtype=floating)
            One dimension array which defines
            the maximal position for each joint
        kwargs : dict
            The argv dict is used to set parameters which have default values
            this are sample_resolution (default = 0.008),
            collision_check (default = True), max_deviation (default = 0.2),
            scale_velocity (default = 1.0) and scale_acceleration (
                default = 1.0)
        """

        try:
            joint_limits = JointLimits(joint_set,
                                       max_s_velocity, max_s_acceleration,
                                       min_position, max_position)
        except (ValueError, TypeError) as exc:
            raise_from(ArgumentError('It was not possible to create'
                                     ' an instance of JointLimts due to'
                                     ' wrong parameter type or format'), exc)

        return cls(move_group_name, JointLimits, kwargs)

    @property
    def move_group_name(self):
        """
        move_group_name : str (read only)
            Name of the move group for which plan parameters are applied
        """
        return self.__move_group_name

    @property
    def joint_limits(self):
        """
        joint_limits: JointLimits(read only)
            define the joint constraints
        """
        return self.__joint_limits

    @property
    def sample_resolution(self):
        """
        sample_resolution: float
            sampling resolution in Hz
        """
        return self.__sample_resolution

    @property
    def collision_check(self):
        """
        collision_check: bool(read only)
            defines if collision check should be performed
        """
        return self.__collision_check

    @property
    def max_deviation(self):
        """
        max_deviation: float(read only)
            defines the maximal deviation from trajectory points
            when fly-by-points in joint space
        """
        return self.__max_deviation

    def __iter__(self):
        return self.__joint_limits.__iter__()

    def __str__(self):
        j_str = '\n'.join(['sampling_resolution = '+str(self.__sample_resolution),
                           'collision_check = '+str(self.__collision_check),
                           'max_devitation = '+str(self.__max_deviation)])
        joint_limts_str = '\n' + self.__joint_limits.__str__()
        joint_limts_str = joint_limts_str.replace('\n', '\n  ')
        return 'PlanParameters:\n' + j_str + joint_limts_str

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        r_tol = 1.0e-13
        a_tol = 1.0e-14

        if not isinstance(other, self.__class__):
            return False

        if id(other) == id(self):
            return True

        if other.joint_limits != self.__joint_limits:
            return False

        if not np.isclose(self.__sample_resolution, other.sampling_resolution,
                          rtol=r_tol, atol=a_tol):
            return False

        if not np.isclose(self.__max_deviation, other.max_deviation,
                          rtol=r_tol, atol=a_tol):
            return False

        if self.__collision_check != other.collision_check:
            return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)
