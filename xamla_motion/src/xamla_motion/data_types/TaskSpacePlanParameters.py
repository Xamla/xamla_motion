#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from future.builtins import *
from future.utils import raise_from, raise_with_traceback
from copy import deepcopy

from data_types import EndEffectorLimits

import numpy as np


class TaskSpacePlanParameters(object):
    """
    TaskSpace holds all constrains for task space trajectory planning

    Attributes
    ----------
    endeffector_name : str (read only)
        Name of the move group for which plan parameters are applied
    endeffector_limits : EndeffectorLimits (read only)
        define the task space constraints
    sample_resolution : float
        sampling resolution in Hz
    collision_check : bool (read only)
        defines if collision check should be performed
    max_deviation : float (read only)
        defines the maximal deviation from trajectory points
        when fly-by-points in task space 
    ik_jump_threshold : float
        defines the inverse kinematic jump threshold
    """

    def __init__(self, *args, **kwargs):
        """
        Initialization of TaskSpacePlanParameters class

        The class can be initialized by a move group name,
        JointSet and velocity and acceleration (xyz and angular) limits or
        by move group name and an instance of EndeffectorLimits because
        TaskSpacePlanParametes is a superset of EndeffectorLimits. Internally
        a instance of EndeffectorLimits is created for the second initialization
        routine. Therefore, please refer to the documentation of EndeffectorLimits
        to properly set the endeffector limits

        Parameters
        ----------
        args : list(str, EndeffectorLimits) or list(str, 4*float)
            If list len is 2 the parameters are interpreted as
            the name of the endeffectpr and an instance of EndeffectorLimits
            If list len is 5 the parameters are interpreted as
            the name of the endeffector, max_xyz_velocity, max_xyz_acceleration, 
            max_angular_velocity, max_angular_acceleration
        argv : dict
            The argv dict is used to set parameters which have default values
            this are sample_resolution (default = 0.008),
            collision_check (default = True), max_deviation (default = 0.2),
            ik_jump_threshold (default = 1.2), scale_velocity (default = 1.0) 
            and scale_acceleration (default = 1.0)

        Yields
        ------
            Instance of TaskSpacePlanParameters

        Raises
        ------
            TypeError : type mismatch
                If only two args arguments are provided and the second is
                not of expected type EndeffectorLimits or if in general the first
                provided argument is not of type str
            ValueError
                If number of args arguments is not equal to two or five
            RuntimeError
                If internally a instance of EndeffectorLimits can not be created
                due to wrong argument size or type

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

        if len(args) == 2:
            if isinstance(args[1], EndeffectorLimits):
                s_x_velocity = (args[1].max_xyz_velocity
                                * self.__scale_velocity)
                s_x_acceleration = (args[1].max_xyz_acceleration
                                    * self.__scale_acceleration)
                s_a_velocity = (args[1].max_angular_velocity
                                * self.__scale_velocity)
                s_a_acceleration = (args[1].max_angular_acceleration
                                    * self.__scale_acceleration)

                self.__endeffector_limits = EndeffectorLimits(args[1].joint_set,
                                                              s_x_velocity,
                                                              s_x_acceleration,
                                                              s_a_velocity,
                                                              s_a_acceleration)
            else:
                raise TypeError('argument 2 (joint_limits) is not'
                                ' of expected type EndeffectorLimits')
        elif len(args) == 5:
            s_x_velocity = args[1] * self.__scale_velocity
            s_x_acceleration = args[2] * self.__scale_acceleration
            s_a_velocity = args[3] * self.__scale_velocity
            s_a_acceleration = args[4] * self.__scale_acceleration
            try:
                self.__endeffector_limits = EndeffectorLimits(s_x_velocity,
                                                              s_x_acceleration,
                                                              s_a_velocity,
                                                              s_a_acceleration)
            except (ValueError, TypeError) as exc:
                raise_from(RuntimeError('It was not possible to create'
                                        ' an instance of Endeffectorlimits'
                                        ' due to wrong parameter'
                                        ' type or size'), exc)
        else:
            raise ValueError('only 2 (endeffector name, instance of'
                             ' EndeffectorLimits) or 5 arguments (endeffector'
                             ' name, and all EndeffectorLimits initialization'
                             ' parameters) are allowed')

        if isinstance(args[0], str):
            self.__endeffector_name = args[0]
        else:
            raise TypeError('argument 1 (endeffector_name) is not '
                            'of expected type str')

    @property
    def endeffector_name(self):
        """get endeffector name read only"""
        return self.__endeffector_name

    @property
    def endeffector_limits(self):
        """get internal endeffector limits instance read only"""
        return self.__endeffector_limits

    @property
    def sample_resolution(self):
        """get used sample resolution value read only"""
        return self.__sample_resolution

    @property
    def collision_check(self):
        """if true collision are checked else this is not the case"""
        return self.__collision_check

    @property
    def max_deviation(self):
        """get value of max devitation which is used in the planning process"""
        return self.__max_deviation

    @property
    def jk_jump_threshold(self):
        """get value of ik jump threshold"""
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
