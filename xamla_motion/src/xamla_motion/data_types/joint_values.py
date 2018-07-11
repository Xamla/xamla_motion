# joint_values.py
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

from functools import total_ordering

import numpy as np

from .joint_set import JointSet
from xamlamoveit_msgs.msg import JointPathPoint


class JointValues(object):
    """
    Manages a set of joints and the respective joint values

    Methods
    -------
    empty()
        Creates empty instance of JointValues
    zero(joint_set)
        Creates instance of JointValues with all values 0.0
    try_get_value(name)
        Tries to get joint value by joint name
    reorder(new_order)
        Creates reordered instance of JointValues by the order of new_order
    transform(transform_function)
        Creates a transformed version of JointValues
    select(names)
        Creates a JointValue instance which only contains selected joints
    from_joint_path_point_msg
        Creates an instance of JointValues from a JointPathPoint ros message
    to_joint_path_point_msg
        Transform to xamlamoveit_msgs JointPathMessage
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

        Returns
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
                    values = [values] * len(self.__joint_set)
                else:
                    values = values * len(self.__joint_set)

            if len(values) != len(joint_set):
                raise ValueError('joint set holds ' +
                                 str(len(self.__joint_set)) +
                                 ' joints but only ' + str(len(values)) +
                                 ' values are provided')
            self.__values = np.fromiter(values, float)
        except (TypeError, ValueError) as exc:
            raise exc

        self.__values.flags.writeable = False

    @classmethod
    def from_joint_path_point_msg(cls, joint_set, msg):
        """
        Creates an instance of JointValues from a JointPathPoint ros message

        Parameters
        ----------
        joint_set : JointSet
            Joint set to assign joint with values
        msg : JointPathPoint ros message
            Message which should be transformed to JointValues

        Returns
        JointValues
            New instance of JointValues

        Raises
        ------
        TypeError
            If joint_set is not of type JointSet
        """

        if not isinstance(joint_set, JointSet):
            raise TypeError('joint_set is not of expected type JointSet')

        return cls(joint_set, msg.positions)

    @staticmethod
    def empty():
        """
        Creates a empty JointValues instance

        Returns
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

        Returns
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
        """
        joint_set : JointSet (readonly)
            A instance of JointSet managing joint names
        """
        return self.__joint_set

    @property
    def values(self):
        """
        valules : numpy.ndarray(dtype=float64) (readonly)
            One dimensional numpy array holding the respective
            joint values
        """
        return self.__values

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
        if not isinstance(joint_name, str):
            raise TypeError('joint_name expected type is str')

        is_found, index = self.__joint_set.try_get_index_of(joint_name)

        if not is_found:
            return False, None
        else:
            return True, self.__values[index]

    def reorder(self, new_order):
        """
        Creates reordered instance of JointValue by the order of new_order

        Parameters
        ----------
        new_order : JointSet
            JointSet which defines the new order

        Returns
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
            raise ValueError('A joint name from new_oder'
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

        Returns
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
        if not (callable(transform_function) or
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
            raise TypeError('wrong transform function format') from exc

        return self.__class__(self.__joint_set, values)

    def select(self, names):
        """
        Creates a JointValue instance which only contains selected joints

        Parameters
        ----------
        names : str or list of str or JointSet
            Joint names which should be in the new JointValues instance


        Returns
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
            raise ValueError('name ' + name +
                             ' not exist in joint names') from exc

    def to_joint_path_point_msg(self):
        """
        Transform to xamlamoveit_msgs JointPathMessage
        """
        joint_path_point = JointPathPoint()
        joint_path_point.positions = list(self.__values)
        return joint_path_point

    def __getitem__(self, key):
        """
        Returns value by joint name or index

        Parameters
        ----------
        key : int ,str or slice
            index of joint, slice or joint name for which the values are requested

        Returns
        -------
        value : numpy floating or numpy ndarray of floating
            Joint value if index or joint name is valid / exists

        Raises
        ------
        TypeError : type mismatch
            If key is not int, str or slice
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

        elif isinstance(key, (int, slice)):
            try:
                return self.__values[key]
            except IndexError as exc:
                raise IndexError('index out of range')
        else:
            raise TypeError(
                'key is not one of expected types int, str or slice ')

    def __len__(self):
        return len(self.__values)

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
                           rtol=r_tol, atol=a_tol,
                           equal_nan=True):
            return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)

    def __add__(self, other):
        if isinstance(other, self.__class__):
            if len(other.joint_set) < len(self.__joint_set):
                values = np.zeros(len(other.joint_set),
                                  self.__values.dtype)
                for i, name in enumerate(other.joint_set):
                    values[i] = (self.__values[self.__joint_set.get_index_of(name)] +
                                 other.values[i])
                return self.__class__(other.joint_set, values)
            else:
                values = np.zeros(
                    len(self.__joint_set), self.__values.dtype)
                for i, name in enumerate(self.__joint_set):
                    values[i] = (self.__values[i] +
                                 other.values[other.joint_set.get_index_of(name)])
        else:
            values = self.__values + other

        return self.__class__(self.__joint_set, values)

    def __radd__(self, other):

        values = other + self.__values
        return self.__class__(self.__joint_set, values)

    def __sub__(self, other):
        if isinstance(other, self.__class__):
            if len(other.joint_set) < len(self.__joint_set):
                values = np.zeros(len(other.joint_set),
                                  self.__values.dtype)
                for i, name in enumerate(other.joint_set):
                    values[i] = (self.__values[self.__joint_set.get_index_of(name)] -
                                 other.values[i])
                return self.__class__(other.joint_set, values)
            else:
                values = np.zeros(
                    len(self.__joint_set), self.__values.dtype)
                for i, name in enumerate(self.__joint_set):
                    values[i] = (self.__values[i] -
                                 other.values[other.joint_set.get_index_of(name)])
        else:
            values = self.__values - other

        return self.__class__(self.__joint_set, values)

    def __rsub__(self, other):
        values = other - self.__values
        return self.__class__(self.__joint_set, values)

    def __mul__(self, other):
        if isinstance(other, self.__class__):
            if len(other.joint_set) < len(self.__joint_set):
                values = np.zeros(len(other.joint_set),
                                  self.__values.dtype)
                for i, name in enumerate(other.joint_set):
                    values[i] = (self.__values[self.__joint_set.get_index_of(name)] *
                                 other.values[i])
                return self.__class__(other.joint_set, values)
            else:
                values = np.zeros(
                    len(self.__joint_set), self.__values.dtype)
                for i, name in enumerate(self.__joint_set):
                    values[i] = (self.__values[i] *
                                 other.values[other.joint_set.get_index_of(name)])
        else:
            values = self.__values * other

        return self.__class__(self.__joint_set, values)

    def __rmul__(self, other):
        values = other * self.__values
        return self.__class__(self.__joint_set, values)

    def __truediv__(self, other):

        if isinstance(other, self.__class__):
            if len(other.joint_set) < len(self.__joint_set):
                values = np.zeros(len(other.joint_set),
                                  self.__values.dtype)
                for i, name in enumerate(other.joint_set):
                    values[i] = (self.__values[self.__joint_set.get_index_of(name)] /
                                 other.values[i])
                return self.__class__(other.joint_set, values)
            else:
                values = np.zeros(
                    len(self.__joint_set), self.__values.dtype)
                for i, name in enumerate(self.__joint_set):
                    values[i] = (self.__values[i] /
                                 other.values[other.joint_set.get_index_of(name)])
        else:
            values = self.__values / other

        return self.__class__(self.__joint_set, values)

    def __rtruediv__(self, other):
        values = other / self.__values
        return self.__class__(self.__joint_set, values)
