# task_space_plan_parameters.py
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

from .end_effector_limits import EndEffectorLimits

import numpy as np

from ..xamla_motion_exceptions import ArgumentError


class TaskSpacePlanParameters(object):
    """
    TaskSpace holds all constrains for task space trajectory planning

    Methods
    -------
    from_arguments(end_effector_name, max_xyz_velocity,
                   max_xyz_acceleration, max_angular_velocity,
                   max_angular_acceleration, **kwargs)
        Creates a instance or TaskSpacePlanParameters
        from single limit arguments
    """

    def __init__(self, end_effector_name, end_effector_limits, **kwargs):
        """
        Initialization of TaskSpacePlanParameters class

        Parameters
        ----------
        end_effector_name : str convertable
            Name of the end effector
        end_effector_limits : EndEffectorLimits
            Limits of the end effector in task space
        kwargs : dict
            The argv dict is used to set parameters which have default values
            this are sample_resolution (default = 0.008 / 125 Hz),
            collision_check (default = True), max_deviation (default = 0.2),
            ik_jump_threshold (default = 1.2)

        Returns
        ------
            Instance of TaskSpacePlanParameters

        Raises
        ------
            TypeError : type mismatch
                If end_effector_limits is not of 
                expected type EndEffectorLimits

        Examples
        --------
        >>> end_effector_limits = EndeffectorLimits(1.0, 1.0, 1.0, 1.0)
        >>> p = TaskSpacePlanParameters('tool1', end_effector_limits, max_deviation=0.1)
        """

        # if you change defaults here please also edit documentation in
        # motion_service.py create_task_space_plan_parameters()
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
            self.__ik_jump_threshold = float(kwargs.get("ik_jump_threshold",
                                                        1.2))
        except TypeError as exc:
            raise TypeError('ik_jump_threshold can not be converted to float')

        try:
            self.__max_deviation = float(kwargs.get("max_deviation",
                                                    0.2))
        except TypeError as exc:
            raise TypeError('max_deviation can not be converted to float')

        if isinstance(end_effector_limits, EndEffectorLimits):
            self.__end_effector_limits = end_effector_limits
        else:
            raise TypeError('end_effector_limits is not of'
                            ' expected type EndEffectorLimits')

        self.__end_effector_name = str(end_effector_name)

    @classmethod
    def from_arguments(cls, end_effector_name, max_xyz_velocity,
                       max_xyz_acceleration, max_angular_velocity,
                       max_angular_acceleration, **kwargs):
        """
        Initialization of TaskSpacePlanParameters by single limits

        Parameters
        ----------
        end_effector_name : str
            Name of the end effector
        max_xyz_velocity : float convertable
            Defines the maximal xyz velocity [m/s]
        max_xyz_acceleration : float convertable (read only)
            Defines the maximal xyz acceleration [m/s^2]
        max_angular_velocity : float convertable (read only)
            Defines the maximal angular velocity [rad/s]
        max_angular_acceleration : float convertable (read only)
            Defines the maximal angular acceleration [rad/s^2]
        kwargs : dict
            The argv dict is used to set parameters which have default values
            this are sample_resolution (default = 0.008),
            collision_check (default = True), max_deviation (default = 0.2),
            ik_jump_threshold (default = 1.2)

        Returns
        -------
        TaskSpacePlanParameters
            Instance of TaskSpacePlanParameters

        Raises
        ------
        ArgumentError
            If instance of EndeffectorLimits could not be created
            due to wrong type or format of the arguments

        Examples
        --------
        >>> p = TaskSpacePlanParameters('tool1', 1.0, 1.0, 1.0, 1.0, max_deviation=0.1)
        """

        try:
            end_effector_limits = EndEffectorLimits(max_xyz_velocity,
                                                    max_xyz_acceleration,
                                                    max_angular_velocity,
                                                    max_angular_acceleration)
        except (ValueError, TypeError) as exc:
            raise ArgumentError('It was not possible to create'
                                ' an instance of Endeffectorlimits'
                                ' due to limit type or size') from exc

        return cls(end_effector_name, end_effector_limits, **kwargs)

    @classmethod
    def from_builder(cls, builder):
        """
        Initilization of TaskSpacePlanParameters from TaskSpacePlanParameters Builder

        Parameters
        ----------
        builder
            Instance of TaskSpacePlanParameter Builder


        Returns
        ------
            Instance of TaskSpacePlanParameters

        Raises
        ------
        TypeError
            If builder is not of expected type TaskSpacePlanParameters Builder
        """

        if not isinstance(builder, cls._Builder):
            raise TypeError('builder is not of expected type '
                            'TaskSpacePlanParameters builder')

        return cls(builder.end_effector_name,
                   builder.e_e_limits,
                   sample_resolution=builder.sample_resolution,
                   collision_check=builder.collision_check,
                   max_deviation=builder.max_deviation,
                   ik_jump_threshold=builder.ik_jump_threshold)

    class _Builder(object):

        def __init__(self, instance):
            self.__end_effector_name = instance.end_effector_name
            self.__e_e_limits = instance.end_effector_limits
            self.__sample_resolution = instance.sample_resolution
            self.__max_deviation = instance.max_deviation
            self.__collision_check = instance.collision_check
            self.__ik_jump_threshold = instance.ik_jump_threshold

        @property
        def end_effector_name(self):
            return self.__end_effector_name

        @property
        def end_effector_limits(self):
            return self.__e_e_limits

        @property
        def sample_resolution(self):
            return self.__sample_resolution

        @sample_resolution.setter
        def sample_resolution(self, value):
            value = float(value)

            self.__sample_resolution = value

        @property
        def collision_check(self):
            return self.__collision_check

        @collision_check.setter
        def collision_check(self, value):
            value = bool(value)

            self.__collision_check = value

        @property
        def max_deviation(self):
            return self.__max_deviation

        @max_deviation.setter
        def max_deviation(self, value):
            value = float(value)

            self.__max_deviation = value

        @property
        def ik_jump_threshold(self):
            return self.__ik_jump_threshold

        @ik_jump_threshold.setter
        def ik_jump_threshold(self, value):
            value = float(value)

            self.__ik_jump_threshold = value

        def scale_velocity(self, value):
            value = float(value)

            if value > 1.0 or value < 0.0:
                raise ValueError('velocity_scaling is not'
                                 ' between 0.0 and 1.0')

            max_xyz_velocity = self.__e_e_limits.max_xyz_velocity
            max_xyz_velocity *= value

            max_angular_velocity = self.__e_e_limits.max_angular_velocity
            max_angular_velocity *= value

            limits = EndEffectorLimits(max_xyz_velocity,
                                       self.__e_e_limits.max_xyz_acceleration,
                                       max_angular_velocity,
                                       self.__e_e_limits.max_angular_acceleration)

            self.__e_e_limits = limits

        def scale_acceleration(self, value):
            value = float(value)

            if value > 1.0 or value < 0.0:
                raise ValueError('acceleration_scaling is not'
                                 ' between 0.0 and 1.0')

            max_xyz_acc = self.__e_e_limits.max_xyz_acceleration
            max_xyz_acc *= value

            max_angular_acc = self.__e_e_limits.max_angular_velocity
            max_angular_acc *= value

            limits = EndEffectorLimits(self.__e_e_limits.max_xyz_velocity,
                                       max_xyz_acc,
                                       self.__e_e_limits.max_angular_velocity,
                                       max_angular_acc)

            self.__e_e_limits = limits

        def build(self):
            return TaskSpacePlanParameters(self.__end_effector_name,
                                           self.__e_e_limits,
                                           sample_resolution=self.__sample_resolution,
                                           collision_check=self.__collision_check,
                                           max_deviation=self.__max_deviation,
                                           ik_jump_threshold=self.__ik_jump_threshold)

    @property
    def end_effector_name(self):
        """
        end_effector_name : str (read only)
            Name of the move group for which plan parameters are applied
        """
        return self.__end_effector_name

    @property
    def end_effector_limits(self):
        """
        end_effector_limits: EndeffectorLimits(read only)
            define the task space constraints
        """
        return self.__end_effector_limits

    @property
    def max_xyz_velocity(self):
        """
        max_xyz_velocity: float(read only)
            max translational velocity
        """
        return self.__end_effector_limits.max_xyz_velocity

    @property
    def max_xyz_acceleration(self):
        """
        max_xyz_acceleration: float(read only)
            max translational acceleration
        """
        return self.__end_effector_limits.max_xyz_acceleration

    @property
    def max_angular_velocity(self):
        """
        max_angular_velocity: float(read only)
            max angular velocity
        """
        return self.__end_effector_limits.max_angular_velocity

    @property
    def max_angular_acceleration(self):
        """
        max_angular_acceleration: float(read only)
            max angular acceleration
        """
        return self.__end_effector_limits.max_angular_acceleration

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

    @property
    def max_deviation(self):
        """
        max_deviation: float
            defines the maximal deviation from trajectory points
            when fly-by-points in joint space
        """
        return self.__max_deviation

    @property
    def ik_jump_threshold(self):
        """
        ik_jump_threshold: float
            defines the inverse kinematic jump threshold
        """
        return self.__ik_jump_threshold

    def with_collision_check(self, value):
        """
        Create an instance of TaskSpacePlanParameters with collision check value

        Parameters
        ----------
        value : bool
            New value of collision check

        Returns
        -------
        TaskSpacePlanParameters
            Instance of TaskSpacePlanParameters with new collision check value

        Raises
        ------
        TypeError
            If value is not of type bool
        """

        if not isinstance(value, bool):
            raise TypeError('value is not of expected type bool')

        if self.__collision_check == value:
            return self
        else:
            builder = self._Builder(self)
            builder.collision_check = value
            return builder.build()

    def with_sample_resolution(self, value):
        """
        Create an instance of TaskSpacePlanParameters with sample resolution value

        Parameters
        ----------
        value : bool
            New value of sample resolution

        Returns
        -------
        TaskSpacePlanParameters
            Instance of TaskSpacePlanParameters with new sample resolution value

        Raises
        ------
        TypeError
            If value is not convertable to float
        """

        value = float(value)

        if np.isclose(self.__sample_resolution, value):
            return self
        else:
            builder = self._Builder(self)
            builder.sample_resolution = value
            return builder.build()

    def with_max_deviation(self, value):
        """
        Create an instance of TaskSpacePlanParameters with max deviation value

        Parameters
        ----------
        value : bool
            New value of max deviation

        Returns
        -------
        TaskSpacePlanParameters
            Instance of TaskSpacePlanParameters with new max deviation value

        Raises
        ------
        TypeError
            If value is not convertable to float
        """

        value = float(value)

        if np.isclose(self.__max_deviation, value):
            return self
        else:
            builder = self._Builder(self)
            builder.max_deviation = value
            return builder.build()

    def with_ik_jump_threshold(self, value):
        """
        Create an instance of TaskSpacePlanParameters with ik jump threshold value

        Parameters
        ----------
        value : bool
            New value of ik jump threshold

        Returns
        -------
        TaskSpacePlanParameters
            Instance of TaskSpacePlanParameters with new ik jump threshold value

        Raises
        ------
        TypeError
            If value is not convertable to float
        """

        value = float(value)

        if np.isclose(self.__max_deviation, value):
            return self
        else:
            builder = self._Builder(self)
            builder.max_deviation = value
            return builder.build()

    def to_builder(self):
        """
        Create an instance of TaskSpacePlanParameters Builder

        Returns
        -------
        TaskSpacePlanParameters Builder
            Instance of TaskSpacePlanParameters Builder
        """

        return self._Builder(self)

    def __str__(self):
        j_str = '\n'.join(['sampling_resolution = '+str(self.__sample_resolution),
                           'collision_check = '+str(self.__collision_check),
                           'max_devitation = '+str(self.__max_deviation),
                           'ik_jump_threshold ='+str(self.__ik_jump_threshold)])
        end_effector_limits_str = '\n' + self.__end_effector_limits.__str__()
        end_effector_limits_str = end_effector_limits_str.replace('\n', '\n  ')
        return 'TaskSpacePlanParameters:\n' + j_str + end_effector_limits_str

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        r_tol = 1.0e-13
        a_tol = 1.0e-14

        if not isinstance(other, self.__class__):
            return False

        if id(other) == id(self):
            return True

        if other.end_effector_limits != self.__end_effector_limits:
            return False

        if not np.isclose(self.__sample_resolution, other.sampling_resolution,
                          rtol=r_tol, atol=a_tol):
            return False

        if not np.isclose(self.__max_deviation, other.max_deviation,
                          rtol=r_tol, atol=a_tol):
            return False

        if not np.isclose(self.__ik_jump_threshold, other.ik_jump_threshold,
                          rtol=r_tol, atol=a_tol):
            return False

        if self.__collision_check != other.collision_check:
            return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)
