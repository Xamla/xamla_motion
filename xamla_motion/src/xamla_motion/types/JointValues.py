#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function)
try:
    from future_builtins import *
except ImportError:
    pass
from functools import total_ordering

from JointSet import JointSet


# @total_ordering
class JointValues(object):
    """
    Class which manages a JointSet and the respective joint values
    """

    def __init__(self, joint_set, values):
        """
        Initialization of the JointValues class

        Parameters
        ----------
        joint_set : JoinSet
            JointSet for which also the values should be managed
        values : float or list of floats
            The JointValue class can be initialized in different ways
            -JointSet + float then all joint values are set to the same
             float value of values
            -JointSet + list of floats then joint values and joint set
             must contain the same number of items and every joint name
             get this specific join value

        Raises
        ------
        TypeError : type mismatch
            If joint_set is not of type JointSet or
            values is not of type float or list of floats
        ValueError : not same size
            If values is list of floats and not contains the same number
            of items as joint_set
        """
        self.__values = list()

        if not isinstance(joint_set, JointSet):
            raise TypeError('joint_set is not expected type JointSet')

        self.__joint_set = joint_set

        if isinstance(values, float):
            self.__values = [values] * joint_set.count()
        elif values and all(isinstance(value, float) for value in values):
            if len(values) != self.joint_set.count():
                raise ValueError('values has not the same size'
                                 ' as JointSet')
            self.__values = values
        else:
            raise TypeError('values is not of expected'
                            ' types float, list[float] or'
                            ' not of same size as JointSet')

    @staticmethod
    def empty():
        """
        Creates a empty JointValues instance

        Returns
        -------
        joint_values : JointValues
            An empty instance of JointValues
        """
        joint_values = JointValues(JointSet.empty(), 0.0)
        joint_values.__values = []
        return joint_values

    @staticmethod
    def zero(joint_set):
        """
        Creates a instance where all joint values are initialized with 0.0

        Parameters
        ----------
        joint_set:
            JointSet which should be managed by JointValues

        Returns
        -------
        joint_values : JointValues
            An JointValue instance with values initialized with 0.0

        Raises
        ------
        TypeError: type mismatch
            If input parameter join_set is not of type JointSet
        """
        if not isinstance(joint_set, JointSet):
            raise TypeError('joint_set is not expected type JointSet')
        joint_values = JointValues(joint_set, 0.0)
        return joint_values

    @property
    def joint_set(self):
        '''read only joint_set'''
        return self.__joint_set

    @property
    def values(self):
        '''read only values'''
        return self.__values

    def count(self):
        """
        Returns number of joints

        Returns
        -------
        count : int
            Number of joints managed by this JointValues
        """
        return len(self.__values)

    def try_get_value(self, joint_name):
        """
        Tries to get joint value by joint name

        Parameters
        ----------
        joint_name : str
            joint name for which it tries to get joint value

        Returns
        -------
        is_found : bool
            If joint name exists and therefore values is found True
            else False
        value : float
            Joint value if joint name existes else None
        """
        if not isinstance(prefix, str):
            raise TypeError('joint_name expected type is str')

        is_found, index = self.__joint_set.try_get_index_of(joint_name)

        if not is_found:
            return False, None
        else:
            return True, self.__value[index]

    def reorder(self, new_order):
        """
        Creates reordered instance of JointValue by the order of new_order

        Parameters
        ----------
        new_order : JointSet
            JointSet which defines the new order

        Returns
        -------
        joint_values : JointValues
            A new Instance of JointValues containing the 
            joints in the order definied by new_order
            (it is ok when new_order only holds a subset of joints)

        Raises
        ------
        TypeError : type mismatch
            If input parameter new_order is not of type JointSet
        ValueError : 
            If joint name from new_order not exists
        """

        if not isinstance(new_order, JointSet):
            raise TypeError('new_order is not of excpeted type JointSet')

        try:
            values = [self.__getitem__(joint_name) for joint_name in new_order]
        except ValueError as exc:
            raise ValueError('joint name ' + joint_name + ' from new_oder'
                             ' not exist in this instance of JointValues')
        return JointValues(new_order, values)

    def transform(self, transform_function):
        if not callable(tranform_function):
            raise TypeError('fransform_function is not callable')

        return JointValues(self.__joint_set,
                           list(map(transform_function, self.__values)))

    def __getitem__(self, item):
        """
        Returns value by joint name or index

        Parameters
        ----------
        item : int or str
            index of joint or joint name for which the value is requested

        Returns
        -------

        value : float
            Joint value if index or joint name is valid / exists

        Raises:
        TypeError : type mismatch
            If item is not int or str
        ValueError :  
            If joint name not exists
        IndexError :
            If index is out of range

        """
        if isinstance(item, str):
            try:
                return self.__values[self.__joint_set.get_index_of(item)]
            except ValueError as exc:
                raise ValueError('[], joint name not exists')

        elif isinstance(item, int):
            try:
                return self.__values[item]
            except IndexError as exc:
                raise IndexError('[], index out of range')
        else:
            raise TypeError('[item] is not one of expected types int or str')
