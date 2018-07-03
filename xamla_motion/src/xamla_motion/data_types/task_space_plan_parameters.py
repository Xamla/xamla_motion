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

#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function)  # , unicode_literals)
#from future.builtins import *
from future.utils import raise_from, raise_with_traceback
from copy import deepcopy

from data_types import EndEffectorLimits

import numpy as np

from xamla_motion_exceptions import ArgumentError


class TaskSpacePlanParameters(object):
    """
    TaskSpace holds all constrains for task space trajectory planning

    Methods
    -------
    from_arguments
        Creates a instance or TaskSpacePlanParameters
        from single limit arguments
    """

    def __init__(self, ende_effector_name, ende_effector_limits, **kwargs):
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
            ik_jump_threshold (default = 1.2), scale_velocity (default = 1.0) 
            and scale_acceleration (default = 1.0)

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
        >>> endeffector_limits = EndeffectorLimits(1.0, 1.0, 1.0, 1.0)
        >>> p = TaskSpacePlanParameters('tool1', endeffector_limits, max_deviation=0.1)
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
        self.__ik_jump_threshold = kwargs.get("ik_jump_threshold", 1.2)
        if not isinstance(self.__ik_jump_threshold, float):
            raise TypeError('ik_jump_threshold is not of expected type float')
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

        if isinstance(ende_effector_limits, EndeffectorLimits):
            s_x_velocity = (ende_effector_limits.max_xyz_velocity
                            * self.__scale_velocity)
            s_x_acceleration = (ende_effector_limits.max_xyz_acceleration
                                * self.__scale_acceleration)
            s_a_velocity = (ende_effector_limits.max_angular_velocity
                            * self.__scale_velocity)
            s_a_acceleration = (ende_effector_limits.max_angular_acceleration
                                * self.__scale_acceleration)

            self.__endeffector_limits = EndeffectorLimits(s_x_velocity,
                                                          s_x_acceleration,
                                                          s_a_velocity,
                                                          s_a_acceleration)
        else:
            raise TypeError('end_effector_limits is not of'
                            ' expected type EndEffectorLimits')

        self.__endeffector_name = str(ende_effector_name)

    @classmethod
    def from_arguments(cls, ende_effector_name, max_xyz_velocity,
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
            ik_jump_threshold (default = 1.2), scale_velocity (default = 1.0) 
            and scale_acceleration (default = 1.0)

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
            endeffector_limits = EndeffectorLimits(max_xyz_velocity,
                                                   max_xyz_acceleration,
                                                   max_angular_velocity,
                                                   max_angular_acceleration)
        except (ValueError, TypeError) as exc:
            raise_from(ArgumentError('It was not possible to create'
                                     ' an instance of Endeffectorlimits'
                                     ' due to limit type or size'), exc)

        return cls(ende_effector_name, endeffector_limits, kwargs)

    @property
    def endeffector_name(self):
        """
        endeffector_name : str (read only)
            Name of the move group for which plan parameters are applied
        """
        return self.__endeffector_name

    @property
    def endeffector_limits(self):
        """
        endeffector_limits: EndeffectorLimits(read only)
            define the task space constraints
        """
        return self.__endeffector_limits

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
            when fly-by-points in task space
        """
        return self.__max_deviation

    @property
    def jk_jump_threshold(self):
        """
        ik_jump_threshold: float
            defines the inverse kinematic jump threshold
        """
        return self.__ik_jump_threshold

    def __str__(self):
        j_str = '\n'.join(['sampling_resolution = '+str(self.__sample_resolution),
                           'collision_check = '+str(self.__collision_check),
                           'max_devitation = '+str(self.__max_deviation),
                           'ik_jump_threshold ='+str(self.__ik_jump_threshold)])
        endeffector_limits_str = '\n' + self.__endeffector_limits.__str__()
        endeffector_limits_str = endeffector_limits_str.replace('\n', '\n  ')
        return 'TaskSpacePlanParameters:\n' + j_str + endeffector_limits_str

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        r_tol = 1.0e-13
        a_tol = 1.0e-14

        if not isinstance(other, self.__class__):
            return False

        if id(other) == id(self):
            return True

        if other.endeffector_limits != self.__endeffector_limits:
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
