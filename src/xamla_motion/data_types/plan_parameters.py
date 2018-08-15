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

#!/usr/bin/env python3

from copy import deepcopy

from .joint_limits import JointLimits
from ..xamla_motion_exceptions import ArgumentError

import numpy as np


class PlanParameters(object):
    """
    PlanParameter holds all constrains for joint space trajectory planning

    Methods
    -------
    from_arguments(move_group_name, joint_set, max_velocity,
                   max_acceleration, min_position, max_position, **kwargs)
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
            this are sample_resolution (default = 0.008 / 125 hz),
            collision_check (default = True), max_deviation (default = 0.2),
            velocity_scaling (default = 1.0) and acceleration_scaling (
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

        # if you change defaults here please also edit documentation in
        # motion_service.py create_plan_parameters()
        try:
            self.__sample_resolution = float(kwargs.get("sample_resolution",
                                                        0.08))
        except TypeError as exc:
            raise TypeError('sample_resolution can not be converted to float')

        try:
            self.__collision_check = bool(kwargs.get("collision_check",
                                                     True))
        except TypeError as exc:
            raise TypeError('collision_check can not be converted to bool')

        try:
            self.__max_deviation = float(kwargs.get("max_deviation",
                                                    0.2))
        except TypeError as exc:
            raise TypeError('max_deviation can not be converted to float')

        try:
            self.__velocity_scaling = float(kwargs.get("velocity_scaling",
                                                       1.0))
        except TypeError as exc:
            raise TypeError('velocity_scaling can not be converted to float')
        if self.__velocity_scaling < 0.0 or self.__velocity_scaling > 1.0:
            raise ValueError('velocity_scaling is not in expected range'
                             'between 0.0 and 1.0')

        try:
            self.__acceleration_scaling = float(kwargs.get(
                "acceleration_scaling",
                1.0))
        except TypeError as exc:
            raise TypeError('acceleration_scaling can not'
                            ' be converted to float')
        if self.__acceleration_scaling < 0.0 or self.__acceleration_scaling > 1.0:
            raise ValueError('acceleration_scaling is not in expected range'
                             'between 0.0 and 1.0')

        if isinstance(joint_limits, JointLimits):
            max_s_velocity = joint_limits.max_velocity * self.__velocity_scaling
            max_s_acceleration = (joint_limits.max_acceleration
                                  * self.__acceleration_scaling)
            self.__o_joint_limits = joint_limits
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
            velocity_scaling (default = 1.0) and acceleration_scaling (
            default = 1.0)
        """

        try:
            joint_limits = JointLimits(joint_set,
                                       max_velocity, max_acceleration,
                                       min_position, max_position)
        except (ValueError, TypeError) as exc:
            raise ArgumentError('It was not possible to create'
                                ' an instance of JointLimts due to'
                                ' wrong parameter type or format') from exc

        return cls(move_group_name, joint_limits, **kwargs)

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
    def joint_set(self):
        """
        joint_set : JointSet (read only)
            set of joints for which the limits are defined
        """
        return self.__joint_limits.joint_set

    @property
    def max_velocity(self):
        """
        max_velocity : numpy.array(dtype=floating) (read only)
            maximal velocities
        """
        return self.__joint_limits.max_velocity

    @property
    def max_acceleration(self):
        """
        max_acceleration : numpy.array(dtype=floating) (read only)
            maximal acceleration
        """
        return self.__joint_limits.max_acceleration

    @property
    def min_position(self):
        """
        min_position : numpy.array(dtype=floating) (read only)
            position limit minimum
        """
        return self.__joint_limits.min_position

    @property
    def max_position(self):
        """
        max_position : numpy.array(dtype=floating) (read only)
            position limit maximum
        """
        return self.__joint_limits.max_position

    @property
    def sample_resolution(self):
        """
        sample_resolution: float
            sampling resolution in Hz
        """
        return self.__sample_resolution

    @sample_resolution.setter
    def sample_resolution(self, value):
        value = float(value)

        self.__sample_resolution = value

    @property
    def collision_check(self):
        """
        collision_check: bool
            defines if collision check should be performed
        """
        return self.__collision_check

    @collision_check.setter
    def collision_check(self, value):
        value = bool(value)

        self.__collision_check = value

    @property
    def max_deviation(self):
        """
        max_deviation: float
            defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        """
        return self.__max_deviation

    @max_deviation.setter
    def max_deviation(self, value):
        value = float(value)

        self.__max_deviation = value

    @property
    def velocity_scaling(self):
        """
        velocity_scaling: float
            scaling factor which was applied to input velocity limits
            range between 0.0 and 1.0
        """
        return self.__velocity_scaling

    @velocity_scaling.setter
    def velocity_scaling(self, value):
        value = float(value)

        if value > 1.0 or value < 0.0:
            raise ValueError('velocity_scaling is not between 0.0 and 1.0')

        max_s_velocity = self.__o_joint_limits.max_velocity * value

        self.__joint_limits = JointLimits(self.__joint_limits.joint_set,
                                          max_s_velocity,
                                          self.__joint_limits.max_acceleration,
                                          self.__joint_limits.min_position,
                                          self.__joint_limits.max_position)

        self.__velocity_scaling = value

    @property
    def acceleration_scaling(self):
        """
        acceleration_scaling: float
            scaling factor which was applied to input acceleration limits
            range between 0.0 and 1.0
        """
        return self.__acceleration_scaling

    @acceleration_scaling.setter
    def acceleration_scaling(self, value):
        value = float(value)

        if value > 1.0 or value < 0.0:
            raise ValueError('acceleration_scaling is not between 0.0 and 1.0')

        max_s_acceleration = self.__o_joint_limits.max_acceleration * value

        self.__joint_limits = JointLimits(self.__joint_limits.joint_set,
                                          self.__joint_limits.max_velocity,
                                          max_s_acceleration,
                                          self.__joint_limits.min_position,
                                          self.__joint_limits.max_position)

        self.__acceleration_scaling = value

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
