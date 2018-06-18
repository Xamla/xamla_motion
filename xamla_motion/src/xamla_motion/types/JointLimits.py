#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from future.builtins import *

import numpy as np

from JointSet import JointSet


class JointLimits(object):
    """
    Manages the joint limits for a set of joints

    Attributes
    ----------
    joint_set : JointSet (readonly)
        A instance of JointSet managing joint names
    max_velocity : list of float or numpy.array(dtype=floating)
        One dimension array which defines the maximal
        velocity for each joint (readonly)
    max_acceleration : list of float or numpy.array(dtype=floating)
        One dimension array which defines the maximal
        acceleration for each joint (readonly)
    min_position : list of float or numpy.array(dtype=floating)
        One dimension array which defines the mininmal
        position for each joint (readonly)
    max_position : list of float or numpy.array(dtype=floating)
        One dimension array which defines the maximal
        position for each joint (readonly)

    Methods
    -------
    select(names)
        Creates a JointLimits instance which only contains selected joints
    """

    def __init__(self, joint_set, max_velocity, max_acceleration,
                 min_position, max_position):
        """
        Initialization of JointLimits class

        Parameters
        ----------
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

        Yields
        ------
        JointLimits
            An instance of class JointLimits

        Raises
        ------
        TypeError : type mismatch
            If joint_set is not of type JointSet or
            max/min values are not of type list of float
            or numpy array of type floating
        ValueError : not same size
            If max/min values are list of float and not contains the same number
            of items as joint_set or numpy array that has not the correct size
        """
        if not isinstance(joint_set, JointSet):
            raise TypeError('joint_set is not of expected type JointSet')

        self.__joint_set = joint_set

        # max_velocity
        if isinstance(max_velocity, np.ndarray):
            if len(max_velocity.shape) != 1:
                raise ValueError('max_velocity is not a one'
                                 ' dimensional numpy array')
            if max_velocity.shape[0] != joint_set.count():
                raise ValueError('max_velocity numpy array has not the '
                                 ' same number of values as'
                                 ' respective joints are present'
                                 ' in joint_set')
            if not issubclass(max_velocity.dtype.type, np.floating):
                raise TypeError('max_velocity dtype is no floating type')

            self.__max_velocity = np.fromiter(max_velocity, float)

        elif ((isinstance(max_velocity, list) or isinstance(max_velocity, tuple))
              and all(isinstance(value, float) for value in max_velocity)):
            if len(max_velocity) != joint_set.count():
                raise ValueError('max_velocity list has not the '
                                 ' same number of values as'
                                 ' respective joints are present'
                                 ' in joint_set')

            self.__max_velocity = np.fromiter(max_velocity, float)
        else:
            raise TypeError('max_velocity is not one of the expected'
                            ' types list of float or numpy array of floating')

        # max_acceleration
        if isinstance(max_acceleration, np.ndarray):
            if len(max_acceleration.shape) != 1:
                raise ValueError('max_acceleration is not a one'
                                 ' dimensional numpy array')
            if max_acceleration.shape[0] != joint_set.count():
                raise ValueError('max_acceleration numpy array has not the '
                                 ' same number of values as'
                                 ' respective joints are present'
                                 ' in joint_set')
            if not issubclass(max_acceleration.dtype.type, np.floating):
                raise TypeError('max_acceleration dtype is no floating type')

            self.__max_acceleration = np.fromiter(max_acceleration, float)

        elif ((isinstance(max_acceleration, list) or isinstance(max_acceleration, tuple))
              and all(isinstance(value, float) for value in max_acceleration)):
            if len(max_acceleration) != joint_set.count():
                raise ValueError('max_acceleration list has not the '
                                 ' same number of values as'
                                 ' respective joints are present'
                                 ' in joint_set')

            self.__max_acceleration = np.fromiter(max_acceleration, float)
        else:
            raise TypeError('max_acceleration is not one of the expected'
                            ' types list of float or numpy array of floating')

        # min_position
        if isinstance(min_position, np.ndarray):
            if len(min_position.shape) != 1:
                raise ValueError('min_position is not a one'
                                 ' dimensional numpy array')
            if min_position.shape[0] != joint_set.count():
                raise ValueError('min_position numpy array has not the '
                                 ' same number of values as'
                                 ' respective joints are present'
                                 ' in joint_set')
            if not issubclass(min_position.dtype.type, np.floating):
                raise TypeError('min_position dtype is no floating type')

            self.__min_position = np.fromiter(min_position, float)

        elif ((isinstance(min_position, list) or isinstance(min_position, tuple))
              and all(isinstance(value, float) for value in min_position)):
            if len(min_position) != joint_set.count():
                raise ValueError('min_position list has not the '
                                 ' same number of values as'
                                 ' respective joints are present'
                                 ' in joint_set')

            self.__min_position = np.fromiter(min_position, float)
        else:
            raise TypeError('min_position is not one of the expected'
                            ' types list of float or numpy array of floating')

        # max_position
        if isinstance(max_position, np.ndarray):
            if len(max_position.shape) != 1:
                raise ValueError('max_position is not a one'
                                 ' dimensional numpy array')
            if max_position.shape[0] != joint_set.count():
                raise ValueError('max_position numpy array has not the '
                                 ' same number of values as'
                                 ' respective joints are present'
                                 ' in joint_set')
            if not issubclass(max_position.dtype.type, np.floating):
                raise TypeError('max_position dtype is no floating type')

            self.__max_position = np.fromiter(max_position, float)

        elif ((isinstance(max_position, list) or isinstance(max_position, tuple))
              and all(isinstance(value, float) for value in max_position)):
            if len(max_position) != joint_set.count():
                raise ValueError('max_position list has not the '
                                 ' same number of values as'
                                 ' respective joints are present'
                                 ' in joint_set')

            self.__max_position = np.fromiter(max_position, float)
        else:
            raise TypeError('max_position is not one of the expected'
                            ' types list of float or numpy array of floating')

        self.__max_velocity.flags.writeable = False
        self.__max_acceleration.flags.writeable = False
        self.__min_position.flags.writeable = False
        self.__max_position.flags.writeable = False

    @property
    def joint_set(self):
        """read only joint_set"""
        return self.__joint_set

    @property
    def max_velocity(self):
        """read only maxmimal velocity"""
        return self.__max_velocity

    @property
    def max_acceleration(self):
        """read only maximal acceleration"""
        return self.__max_acceleration

    @property
    def min_position(self):
        """read only min position"""
        return self.__min_position

    @property
    def max_position(self):
        """read only max position"""
        return self.__max_position

    def select(self, names):
        """
        Creates a JointLimits instance which only contains selected joints

        Parameters
        ----------
        names : str or list of str
            Joint names which from the new JointLimit instance


        Yields
        ------
        JointLimits
            New instance of JointLimits with selected joints

        Raises
        ------
        TypeError : type mismatch
            If names is not of type str or list of str
        ValueError :
            If name not exist in joint names
        """
        try:
            if isinstance(names, str):
                names = (names)

            if names and all(isinstance(s, str) for s in names):
                max_velocity = np.zeros(len(names), self.__max_velocity.dtype)
                max_acceleration = np.zeros(
                    len(names), self.__max_acceleration.dtype)
                min_position = np.zeros(len(names), self.__min_position.dtype)
                max_position = np.zeros(len(names), self.__max_position.dtype)
                for i, name in enumerate(names):
                    index = self.__joint_set.get_index_of(name)
                    max_velocity[i] = self.__max_velocity[index]
                    max_acceleration[i] = self.__max_acceleration[index]
                    min_position[i] = self.__min_position[index]
                    max_position[i] = self.__max_position[index]
            else:
                raise TypeError('names is not one of the expected types'
                                ' str or list of str')
            return self.__class__(JointSet(names), max_velocity, max_acceleration,
                                  min_position, max_position)
        except ValueError as exc:
            raise_from(ValueError('joint name ' + name +
                                  ' not exist in joint names'), exc)

    def __iter__(self):
        return (self.__max_velocity.__iter__(),
                self.__max_acceleration.__iter__(),
                self.__min_position.__iter__(),
                self.__max_position.__iter__())

    def __str__(self):
        joint_limits_str = '\n'.join([name + ' :  max velocity=' +
                                      str(self.__max_velocity[i]) + ' max acceleration=' +
                                      str(self.__max_acceleration[i]) + ' min position=' +
                                      str(self.__min_position[i]) + ' max position=' +
                                      str(self.__max_position[i])
                                      for i, name in enumerate(self.__joint_set)])
        return 'JointLimits:\n' + joint_limits_str

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        r_tol = 1.0e-13
        a_tol = 1.0e-14

        if not isinstance(other, self.__class__):
            return False

        if id(other) == id(self):
            return True

        if other.join_set != self.__joint_set:
            return False

        if not np.allclose(self.__max_velocity, other.max_velocity,
                           rtol=r_tol, atol=a_tol):
            return False

        if not np.allclose(self.__max_acceleration, other.max_acceleration,
                           rtol=r_tol, atol=a_tol):
            return False

        if not np.allclose(self.__min_position, other.min_position,
                           rtol=r_tol, atol=a_tol):
            return False

        if not np.allclose(self.__max_position, other.max_position,
                           rtol=r_tol, atol=a_tol):
            return False
        return True

    def __ne__(self, other):
        return not self.__eq__(other)
