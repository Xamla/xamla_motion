# joint_set.py
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
                        print_function)  # ,  unicode_literals)
#from future.builtins import *
from functools import total_ordering


@total_ordering
class JointSet(object):
    """
    Manages a list of joint names

    Methods
    -------
    empty()
        Creates emtpy JointSet
    add_prefix(prefix)
        Creates new JointSet where prefix is added to every joint name
    is_subset(other)
        Checks if it is a subset of the JointSet in parameter other
    is_similar(other)
        Checks if it contains the same joint names as in JointSet other
    try_get_index_of(name)
        Tries to get the list index of the joint by name
    get_index_of(name)
        Returns the list index of joint name specificed paramerter name
    contains(name)
        Checks if this JointSet contains a specific joint name
    """

    def __init__(self, names):
        """
        Initialization of the JointSet class

        Parameters
        ----------
        names : str or Iterable[str convertable]
            The Joint set class is initializable in different ways.
            -by a string which only contains one joint name
            -by a string which contains multiple joints names
             separated by a comma as delimiter
            -Iterable container where each time is convertable to str

        Returns
        ------
            An instance of class JointSet

        Raises
        ------
        TypeError : type mismatch
            If input parameter names is not of type str or Iterable[str convertable]
        """

        if isinstance(names, str):
            self.__names = list(s.strip() for s in names.split(','))
        else:
            self.__names = list()
            for name in names:
                self.__names.append(str(name))

    @staticmethod
    def empty():
        """
        Creates a empty JointSet

        Returns
        ------
        JointSet
            The created empty JointSet
        """
        joint_set = JointSet('')
        joint_set.__JointSet__names = list()
        return joint_set

    @property
    def names(self):
        """
        names : List[str] (readonly)
            List of joint names
        """
        return self.__names

    def add_prefix(prefix):
        """
        Creates new JointSet where prefix is added to every joint name

        Parameters
        ----------
        prefix : str
            Defines the prefix which is added in front of every joint name


        Raises
        ------
        TypeError : type mismatch
            If input parameter prefix is not of type str

        Returns
        ------
        JointSet
            The created JointSet with added prefix to joint names

        """
        if not isinstance(prefix, str):
            raise TypeError('prefix expected type is str')
        names = list(map(lambda x: prefix + x, self.__names))
        return self.__class__(names)

    def is_subset(self, other):
        """
        Checks if it is a subset of the JointSet in parameter other

        Parameters
        ----------
        other : JointSet
            JointSet which is used for comparision

        Returns
        -------
        result : bool
            If this JointSet is a subset of the other JointSet
            returns True else False

        Raises
        ------
        TypeError : type mismatch
            If the parameter other is not of type JointSet
        """
        if not isinstance(other, self.__class__):
            raise TypeError('other expected type is JointSet')

        return all(name in other.__names for name in self.__names)

    def is_similar(self, other):
        """
        Checks if it contains the same joint names as in JointSet other

        The method returns true if all joint names are available in the
        other join set. It is not necessary that both have the same order.

        Parameters
        ----------
        other : JointSet
            JointSet which is used for comparision

        Returns
        -------
        result : bool
            If this JointSet the joint names and number of joint names
            returns True else False

        Raises
        ------
        TypeError : type mismatch
            If the parameter other is not of type JointSet
        """
        if not isinstance(other, self.__class__):
            raise TypeError('other expected type is JointSet')

        return len(other) == len(self.__names) and self.is_subset(other)

    def try_get_index_of(self, name):
        """
        Tries to get the list index of the joint by name

        Parameters
        ----------
        name : str
            joint name for which it tries to find the list index

        Results
        -------
        is_found : bool
            If the index is found it resturns True else False
        index : int
            If index is found returns the index else returns None

        Raises
        ------
        TypeError : type mismatch
            If parameter name is not type of str
        """
        if not isinstance(name, str):
            raise TypeError('name expected is type str')

        try:
            return True, self.__names.index(name)
        except ValueError as exc:
            return False, None

    def get_index_of(self, name):
        """
        Returns the list index of joint name specificed paramerter name

        Parameter
        ---------
        name : str
            joint name for which it finds the list index

        Returns
        -------
        index : int
            List index of the searched joint name

        Raises:
        ------
        TypeError : type mismatch
            If parameter name is not type of str
        ValueError : value not exists
            If joint name not exists
        """
        if not isinstance(name, str):
            raise TypeError('name expected type is str')

        try:
            return self.__names.index(name)
        except ValueError as exc:
            raise ValueError('get_index_of: %s', exc)

    def contains(self, name):
        """
        Checks if this JointSet contains a specific joint name

        Parameters
        ----------
        name : str
            joint name for which it checks that it is available

        Returns
        -------
        is_found : bool
            If joint name specified by name is found retruns
            True else False

        Raises
        ------
        TypeError : type mismatch
            If parameter name is not type of str
        """
        if not isinstance(index, str):
            raise TypeError('name expected type is str')

        is_found, _ = self.try_get_index_of(name)

        return is_found

    def __getitem__(self, index):
        """
        Returns the joint name stored at index

        Parameters
        ----------
        index : int
            list index where to get the joint name from

        Returns
        -------
        joint_name : str
            Retruns the joint name at list index specifed by index

        Raises
        ------
        TypeError : type mismatch
            If parameter index is not type of int
        IndexError : index out of range
            If index is out of internal joint name list range
        """
        if not isinstance(index, int):
            raise TypeError('index expected type is int')

        try:
            return self.__names[index]
        except IndexError as exc:
            raise IndexError('index out of range')
        return self.__names[index]

    def __len__(self):
        return len(self.__names)

    def __iter__(self):
        return self.__names.__iter__()

    def __str__(self):
        return 'JointSet:\n'+'\n'.join(self.__names)

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False

        if id(other) == id(self):
            return True

        for i, name in enumerate(other):
            if self.__names[i] != name:
                return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        if not isinstance(other, self.__class__):
            return False

        if len(other) != len(self.__names) and self.is_subset(other):
            return True
        else:
            return False
