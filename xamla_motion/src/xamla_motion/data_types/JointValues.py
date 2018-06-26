#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from future.builtins import *
from future.utils import raise_from, raise_with_traceback
from functools import total_ordering

import numpy as np

from data_types import JointSet


class JointValues(object):
    """
    Manages a set of joints and the respective joint values

    Attributes
    ----------
    joint_set : JointSet (readonly)
        A instance of JointSet managing joint names
    valules : numpy.ndarray(dtype=float64) (readonly)
        One dimensional numpy array holding the respective
        joint values

    Methods
    -------
    empty()
        Creates empty instance of JointValues
    zero(joint_set)
        Creates instance of JointValues with all values 0.0
    count()
        Returns the number of joints managed by JointValues
    try_get_value(name)
        Tries to get joint value by joint name
    reorder(new_order)
        Creates reordered instance of JointValues by the order of new_order
    transform(transform_function)
        Creates a transformed version of JointValues
    select(names)
        Creates a JointValue instance which only contains selected joints
    """

    def __init__(self, joint_set, values):
        """
        Initialization of the JointValues class

        Parameters
        ----------
        joint_set : JoinSet
            JointSet for which also the values should be managed
        values : iterable[float castable]
            The JointValue class can be initialized in different ways
            -JointSet + only on float convertable value then all
             joint values are set to the same float value
            -JointSet + an iterable type with the same number of
             items as number of joints in joint set. (mapping one
             to one)

        Yields
        ------
        JointValues
            An instance of class JointValues

        Raises
        ------
        TypeError : type mismatch
            If joint_set is not of type JointSet or
            type in values is not float castable
        ValueError : not same size
            If values is list of floats and not contains the same number
            of items as joint_set or numpy array that has not the correct size
        """
        self.__values = np.array([])

        if not isinstance(joint_set, JointSet):
            raise TypeError('joint_set is not expected type JointSet')

        self.__joint_set = joint_set

        try:
            try:
                if len(values) <= 1:
                    raise ValueError('max one value is provided')
            except (TypeError, ValueError) as exc:
                if isinstance(exc, TypeError):
                    values = [values] * self.__joint_set.count()
                else:
                    values = values * self.__joint_set.count()

            if len(values) != joint_set.count():
                raise ValueError('joint set holds ' +
                                 str(self.__joint_set.count()) +
                                 ' joints but only ' + str(len(values)) +
                                 ' values are provided')
            self.__values = np.fromiter(values, float)
        except (TypeError, ValueError) as exc:
            raise exc

        self.__values.flags.writeable = False

    @staticmethod
    def empty():
        """
        Creates a empty JointValues instance

        Yields
        ------
        joint_values : JointValues
            An empty instance of JointValues
        """
        joint_values = JointValues(JointSet.empty(), 0.0)
        joint_values.__values = np.array([])
        return joint_values

    @staticmethod
    def zero(self, joint_set):
        """
        Creates instance of JointValues with all values 0.0

        Parameters
        ----------
        joint_set:
            JointSet which should be managed by JointValues

        Yields
        ------
        JointValues
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
        """read only joint_set"""
        return self.__joint_set

    @property
    def values(self):
        """read only values"""
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

        Yields
        ------
        JointValues
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
        return self.__class__(new_order, values)

    def transform(self, transform_function):
        """
        Creates a transformed version of JointValues

        The transformation which is applied to every value in
        JointValues is defined by the transform function

        Parameters
        ----------
        transform_function : callable or numpy.ufunc
            Function which is applied to every value in join values

        Yields
        ------
        JointValues
            A new Instance of JointValues with transformed
            joint values

        Raises
        ------
        TypeError : type mismatch
            If transform function is not callable or not
            a numpy.ufunc and if the function dont has the
            signature input : floating , output : floating
        """
        if not (callable(tranform_function) or
                isinstance(transform_function, np.ufunc)):
            raise TypeError('transform_function is not callable'
                            ' or no numpy ufunc')

        try:
            if isinstance(transform_function, np.ufunc):
                values = transform_function(self.__values)
                if values.shape[0] != self.__values.shape[0]:
                    raise TypeError('function is not a one to on '
                                    ' mapping function')
            else:
                values = np.fromiter(map(transform_function, self.__values),
                                     self.__values.dtype)
        except TypeError as exc:
            raise_from(TypeError('wrong transform function format'))

        return self.__class__(self.__joint_set, value)

    def select(self, names):
        """
        Creates a JointValue instance which only contains selected joints

        Parameters
        ----------
        names : str or list of str or JointSet
            Joint names which should be in the new JointValues instance


        Yields
        ------
        JointValues
            New instance of JointValues with selected joints

        Raises
        ------
        TypeError : type mismatch
            If names is not of type str or list of str
        ValueError :
            If name not exist in joint names
        """
        try:
            if isinstance(names, JointSet):
                return self.reorder(names)
            elif isinstance(names, str):
                values = self.__values[self.__joint_set.get_index_of(names)]
            elif names and all(isinstance(s, str) for s in names):
                values = np.zeros(len(names), self.__values.dtype)
                for i, name in enumerate(names):
                    values[i] = self.__values[self.__joint_set.get_index_of(
                        name)]
            else:
                raise TypeError('names is not one of the expected types'
                                ' str or list of strs')
            return self.__class__(JointSet(names), values)
        except ValueError as exc:
            raise_from(ValueError('name ' + name +
                                  ' not exist in joint names'), exc)

    def __getitem__(self, key):
        """
        Returns value by joint name or index

        Parameters
        ----------
        key : int or str
            index of joint or joint name for which the value is requested

        Returns
        -------

        value : float
            Joint value if index or joint name is valid / exists

        Raises:
        TypeError : type mismatch
            If key is not int or str
        ValueError :
            If joint name not exists
        IndexError :
            If index is out of range

        """
        if isinstance(key, str):
            try:
                return self.__values[self.__joint_set.get_index_of(key)]
            except ValueError as exc:
                raise ValueError('joint name not exists')

        elif isinstance(key, int):
            try:
                return self.__values[key]
            except IndexError as exc:
                raise IndexError('index out of range')
        else:
            raise TypeError('key is not one of expected types int or str')

    def __iter__(self):
        return self.__values.__iter__()

    def __str__(self):
        values_str = '\n'.join([name + ' : ' + str(self.__values[i])
                                for i, name in enumerate(self.__joint_set)])
        return 'JointValues:\n' + values_str

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        r_tol = 1.0e-13
        a_tol = 1.0e-14

        if not isinstance(other, JointValues):
            return False

        if id(other) == id(self):
            return True

        if other.join_set != self.__joint_set:
            return False

        if not np.allclose(self.values, other.values,
                           rtol=r_tol, atol=a_tol):
            return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)

    def __add__(self, other):
        try:
            if isinstance(other, self.__class__):
                if other.joint_set.count() < self.__joint_set.count():
                    values = np.zeros(other.joint_set.count(),
                                      self.__values.dtype)
                    for i, name in enumerate(other.joint_set):
                        values[i] = (self.__values[self.__joint_set.get_index_of(name)] +
                                     other.values[i])
                    return self.__class__(other.joint_set, values)
                else:
                    values = np.zeros(
                        self.__joint_set.count(), self.__values.dtype)
                    for i, name in enumerate(self.__joint_set):
                        values[i] = (self.__values[i] +
                                     other.values[other.joint_set.get_index_of(name)])
            else:
                values = self.__values + other

            return self.__class__(self.__joint_set, values)
        except Exception as exc:
            raise_from(RuntimeError('addition could not be performed'), exc)

    def __radd__(self, other):
        try:
            values = other + self.__values
            return self.__class__(self.__joint_set, values)
        except Exception as exc:
            raise_from(RuntimeError(
                'addition could not be preformed'), exc)

    def __sub__(self, other):
        try:
            if isinstance(other, self.__class__):
                if other.joint_set.count() < self.__joint_set.count():
                    values = np.zeros(other.joint_set.count(),
                                      self.__values.dtype)
                    for i, name in enumerate(other.joint_set):
                        values[i] = (self.__values[self.__joint_set.get_index_of(name)] -
                                     other.values[i])
                    return self.__class__(other.joint_set, values)
                else:
                    values = np.zeros(
                        self.__joint_set.count(), self.__values.dtype)
                    for i, name in enumerate(self.__joint_set):
                        values[i] = (self.__values[i] -
                                     other.values[other.joint_set.get_index_of(name)])
            else:
                values = self.__values - other

            return self.__class__(self.__joint_set, values)
        except Exception as exc:
            raise_from(RuntimeError(
                'substraction could not be performed'), exc)

    def __rsub__(self, other):
        try:
            values = other - self.__values
            return self.__class__(self.__joint_set, values)
        except Exception as exc:
            raise_from(RuntimeError(
                'substraction could not be preformed'), exc)

    def __mul__(self, other):
        try:
            if isinstance(other, self.__class__):
                if other.joint_set.count() < self.__joint_set.count():
                    values = np.zeros(other.joint_set.count(),
                                      self.__values.dtype)
                    for i, name in enumerate(other.joint_set):
                        values[i] = (self.__values[self.__joint_set.get_index_of(name)] *
                                     other.values[i])
                    return self.__class__(other.joint_set, values)
                else:
                    values = np.zeros(
                        self.__joint_set.count(), self.__values.dtype)
                    for i, name in enumerate(self.__joint_set):
                        values[i] = (self.__values[i] *
                                     other.values[other.joint_set.get_index_of(name)])
            else:
                values = self.__values * other

            return self.__class__(self.__joint_set, values)
        except Exception as exc:
            raise_from(RuntimeError(
                'multiplication could not be performed'), exc)

    def __rmul__(self, other):
        try:
            values = other * self.__values
            return self.__class__(self.__joint_set, values)
        except Exception as exc:
            raise_from(RuntimeError(
                'multiplication could not be preformed'), exc)

    def __truediv__(self, other):
        try:
            if isinstance(other, self.__class__):
                if other.joint_set.count() < self.__joint_set.count():
                    values = np.zeros(other.joint_set.count(),
                                      self.__values.dtype)
                    for i, name in enumerate(other.joint_set):
                        values[i] = (self.__values[self.__joint_set.get_index_of(name)] /
                                     other.values[i])
                    return self.__class__(other.joint_set, values)
                else:
                    values = np.zeros(
                        self.__joint_set.count(), self.__values.dtype)
                    for i, name in enumerate(self.__joint_set):
                        values[i] = (self.__values[i] /
                                     other.values[other.joint_set.get_index_of(name)])
            else:
                values = self.__values / other

            return self.__class__(self.__joint_set, values)
        except Exception as exc:
            raise_from(RuntimeError(
                'division could not be performed'), exc)

    def __rtruediv__(self, other):
        try:
            values = other / self.__values
            return self.__class__(self.__joint_set, values)
        except Exception as exc:
            raise_from(RuntimeError(
                'division could not be preformed'), exc)
