#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from future.builtins import *
from future.utils import raise_from, raise_with_traceback
from copy import deepcopy

from JointLimits import JointLimits

import numpy as np


def PlanParameters(object):

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
            collision_check (default = True) and max_deviation (default = 0.2) 

        Yields
        ------
            Instance of PlanParameters
        """

        if len(args) == 2:
            if isinstance(args[1], JointLimits):
                self.__join_limits = deepcopy(args[1])
            else:
                raise TypeError('argument 2 (joint_limits) is not'
                                ' of expected type JointLimits')
        elif len(args) == 6:

        else:
            raise ValueError('only 2 or 6 arguments are allowed')

        if isinstance(args[0], str):
            self.__move_group_name = args[0]
        else:
            raise TypeError('argument 1 (move_group_name) is not '
                            'of expected type str')
