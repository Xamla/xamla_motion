#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from future.builtins import *
from future.utils import raise_from, raise_with_traceback
from copy import deepcopy

from data_types import JointLimits

import numpy as np


class PlanParameters(object):
    """
    PlanParameter holds all constrains for joint space trajectory planning

    Attributes
    ----------
    move_group_name : str (read only)
        Name of the move group for which plan parameters are applied
    joint_limits : JointLimits (read only)
        define the joint constraints
    sample_resolution : float
        sampling resolution in Hz
    collision_check : bool (read only)
        defines if collision check should be performed
    max_deviation : float (read only)
        defines the maximal deviation from trajectory points
        when fly-by-points in joint space 
    """

    def __init__(self, *args, **kwargs):
        """
        Initialization of PlanParameters class

        The class can be initialized by a move_group_name,
        JointSet and velocity, acceleration and position limits or
        by move_group_name and an instance of JointLimits because
        PlanParametes is a superset of JointLimits. Internally
        a instance of JointLimits is created for the second initialization
        routine. Therefore, please refer to the documentation of JointLimits
        to properly set the joint limits

        Parameters
        ----------
        args : list(str, JointLimits) or list(str, JointSet, 4*numpy.ndarray)
            If list len is 2 the parameters are interpreted as
            the name of the move group and an instance of JointLimits
            If list len is 6 the parameters are interpreted as
            the name of the move group, an instance of joint_set,
            max_velocity, max_acceleration, min_position, max_position
        argv : dict
            The argv dict is used to set parameters which have default values
            this are sample_resolution (default = 0.008),
            collision_check (default = True), max_deviation (default = 0.2),
            scale_velocity (default = 1.0) and scale_acceleration (
                default = 1.0)

        Yields
        ------
            Instance of PlanParameters

        Raises
        ------
            TypeError : type mismatch
                If only two args arguments are provided and the second is
                not of expected type JointLimts or if in general the first
                provided argument is not of type str
            ValueError
                If number of args arguments is not equal to two or six
            RuntimeError
                If internally a instance of JointLimits can not be created
                due to wrong argument size or type

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

        if len(args) == 2:
            if isinstance(args[1], JointLimits):
                max_s_velocity = args[1].max_velocity * self.__scale_velocity
                max_s_acceleration = (args[1].max_acceleration
                                      * self.__scale_acceleration)
                self.__joint_limits = JointLimits(args[1].joint_set,
                                                  max_s_velocity,
                                                  max_s_acceleration,
                                                  args[1].min_position,
                                                  args[1].max_position)
            else:
                raise TypeError('argument 2 (joint_limits) is not'
                                ' of expected type JointLimits')
        elif len(args) == 6:
            max_s_velocity = args[2] * self.__scale_velocity
            max_s_acceleration = (args[3]
                                  * self.__scale_acceleration)
            try:
                self.__joint_limits = JointLimits(args[1],
                                                  max_s_velocity, max_s_acceleration,
                                                  args[4], args[5])
            except (ValueError, TypeError) as exc:
                raise_from(RuntimeError('It was not possible to create'
                                        ' an instance of JointLimts due to'
                                        ' wrong parameter type or size'), exc)
        else:
            raise ValueError('only 2 (move_group_name, instance of'
                             ' JointLimits) or 6 arguments (move_group_name,'
                             ' and all JointLimits initialization parameters)'
                             ' are allowed')

        if isinstance(args[0], str):
            self.__move_group_name = args[0]
        else:
            raise TypeError('argument 1 (move_group_name) is not '
                            'of expected type str')

    @property
    def move_group_name(self):
        """get move group name read only"""
        return self.__move_group_name

    @property
    def joint_limits(self):
        """get internal joint limits instance read only"""
        return self.__joint_limits

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
