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

#!/usr/bin/env python3

from functools import total_ordering
from typing import Iterable


@total_ordering
class JointSet(object):
    """
    Manages a list of unique joint names

    Methods
    -------
    empty()
        Creates emtpy JointSet
    add_prefix(prefix)
        Creates new JointSet where prefix is added to every joint name
    is_subset(other)
        Checks if it is a subset of the JointSet in parameter other
    is_superset(other)
        Checks if it is a superset of the JointSet in parameter other
    is_similar(other)
        Checks if it contains the same joint names as in JointSet other
    union(other)
        Creates new JointSet which contains the union of self and other joints
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

        Examples
        --------
        Create an instance of JointSet by string and iterable type

        >>> from xamla_motion.data_types import JointSet
        >>> JointSet('Joint1, Joint2')
        JointSet:
        Joint1
        Joint2
        >>> JointSet(['Joint1','Joint2'])
        JointSet:
        Joint1
        Joint2
        >>> JointSet('Joint1,Joint1,Joint2')
        JointSet:
        Joint1
        Joint2

        """

        self._names = []
        self._names_set = set()

        if isinstance(names, str):
            names = map(lambda x: x.strip(), names.split(','))

        for name in names:
            if name not in self._names_set:
                self._names_set.add(str(name))
                self._names.append(str(name))

        self._names = tuple(self._names)

    @staticmethod
    def empty():
        """
        Creates a empty JointSet

        Returns
        ------
        JointSet
            The created empty JointSet

        Examples
        --------
        Create a empty JointSet instance

        >>> from xamla_motion.data_types import JointSet
        >>> JointSet.empty()
        JointSet:

        """
        joint_set = JointSet('')
        joint_set.__JointSet__names_set = set()
        joint_set.__JointSet__names = tuple()
        return joint_set

    @property
    def names(self):
        """
        names : List[str] (readonly)
            List of joint names
        """
        return list(self._names)

    def add_prefix(self, prefix):
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

        Examples
        --------
        Create new JointSet instance with added prefix

        >>> from xamla_motion.data_types import JointSet
        >>> joint_set = JointSet('joint1, joint2')
        >>> joint_set.add_prefix('robot1_')
        JointSet:
        robot1_joint1
        robot1_joint2

        """
        if not isinstance(prefix, str):
            raise TypeError('prefix expected type is str')
        names = list(map(lambda x: prefix + x, self._names))
        return self.__class__(names)

    def union(self, others):
        """
        Creates new JointSet which contains the union of self and others joints

        Parameters
        ----------
        others : JointSet or Iterable[JointSet]
            JointSet with which the union is performed

        Raises
        ------
        TypeError : type mismatch
            If others is not one of expected types
            JointSet or Iterable of JointSet
        Returns
        ------
        JointSet
            The created JointSet with union joint names

        Examples
        --------
        Create new JointSet instance which is the union of two existing ones

        >>> from xamla_motion.data_types import JointSet
        >>> joint_set1 = JointSet('joint1, joint2')
        >>> joint_set2 = JointSet('joint2, joint3')
        >>> joint_set1.union(joint_set2)
        JointSet:
        joint1
        joint2
        joint3

        """

        names = list(self._names)
        names_set = set(self._names_set)

        if isinstance(others, JointSet):
            for name in others.names:
                if name not in self._names_set:
                    names.append(name)
        elif all(isinstance(i, JointSet) for i in others):
            for other in others:
                for name in other.names:
                    if name not in names_set:
                        names.append(name)
                        names_set.add(name)
        else:
            raise TypeError('others is not one of expected types'
                            ' JointSet or Iterable of JointSet')

        return self.__class__(names)

    def intersection(self, others):
        """
        Creates new JointSet which contains the intersection of self and others joints

        Parameters
        ----------
        others : JointSet or Iterable[JointSet]
            JointSet with which the union is performed

        Raises
        ------
        TypeError : type mismatch
            If others is not one of expected types
            JointSet or Iterable of JointSet

        Returns
        ------
        JointSet
            The created JointSet with intersecting joint names

        Examples
        --------
        Create new JointSet instance which is the intersection of two existing ones

        >>> from xamla_motion.data_types import JointSet
        >>> joint_set1 = JointSet('joint1, joint2')
        >>> joint_set2 = JointSet('joint2')
        >>> joint_set1.intersection(joint_set2)
        JointSet:
        joint2

        """

        names = []

        if isinstance(others, JointSet):
            intersection_names = self._names_set.intersection(
                others._names_set)
        elif all(isinstance(i, JointSet) for i in others):
            intersection_names = self._names_set.intersection(
                o._names_set for o in others)
        else:
            raise TypeError('others is not one of expected types'
                            ' JointSet or Iterable of JointSet')

        for name in self._names:
            if name in intersection_names:
                names.append(name)

        return self.__class__(names)

    def difference(self, others):
        """
        Creates new JointSet which contains the union of self and others joints

        Parameters
        ----------
        others : JointSet
            JointSet with which the union is performed

        Raises
        ------
        TypeError : type mismatch
            If others is not one of expected type JointSet
        Returns
        ------
        JointSet
            The created JointSet with union joint names

        Examples
        --------
        Create new JointSet instance which is the union of two existing ones

        >>> from xamla_motion.data_types import JointSet
        >>> joint_set1 = JointSet('joint1, joint2, joint3')
        >>> joint_set2 = JointSet('joint1, joint2')
        >>> joint_set1.difference(joint_set2)
        JointSet:
        joint3

        """

        names = []

        if isinstance(others, JointSet):
            diff = self._names_set.difference(others._names_set)
            for name in self._names:
                if name in diff:
                    names.append(name)
        else:
            raise TypeError('others is not one of expected types'
                            ' JointSet or Iterable of JointSet')

        return self.__class__(names)

    def is_subset(self, other):
        """
        Checks if it is a subset of the JointSet in parameter other

        The method returns True if it is a subset or similar to other.
        The ordering of the joint names is not checked.

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

        Examples
        --------
        Check if one joint set is the subset of another

        >>> from xamla_motion.data_types import JointSet
        >>> joint_set1 = JointSet('joint1, joint2, joint3')
        >>> joint_set2 = JointSet('joint1, joint2')
        >>> joint_set2.is_subset(joint_set1)
        True

        """
        if not isinstance(other, self.__class__):
            raise TypeError('other has not expected type JointSet')

        return self._names_set.issubset(other._names_set)

    def is_superset(self, other):
        """
        Checks if it is a superset of the JointSet in parameter other

        The method returns True if it is a superset or similar to other.
        The ordering of the joint names is not checked.

        Parameters
        ----------
        other : JointSet
            JointSet which is used for comparision

        Returns
        -------
        result : bool
            If this JointSet is a superset of the other JointSet
            returns True else False

        Raises
        ------
        TypeError : type mismatch
            If the parameter other is not of type JointSet

        Examples
        --------
        Check if one joint set is the superset of another

        >>> from xamla_motion.data_types import JointSet
        >>> joint_set1 = JointSet('joint1, joint2, joint3')
        >>> joint_set2 = JointSet('joint1, joint2')
        >>> joint_set1.is_superset(joint_set2)
        True

        """
        if not isinstance(other, self.__class__):
            raise TypeError('other has not expected type JointSet')

        return self._names_set.issuperset(other._names_set)

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

        Examples
        --------
        Check if one joint set is the similar to another

        >>> from xamla_motion.data_types import JointSet
        >>> joint_set1 = JointSet('joint1, joint2, joint3')
        >>> joint_set2 = JointSet('joint1, joint3, joint2')
        >>> joint_set3 = JointSet('joint1, joint3')
        >>> joint_set1.is_similar(joint_set2)
        True
        >>> joint_set3.is_similar(joint_set1)
        False

        """
        if not isinstance(other, self.__class__):
            raise TypeError('other has not expected type JointSet')

        return len(other) == len(self._names) and self.is_subset(other)

    def try_get_index_of(self, name):
        """
        Tries to get the list index of the joint by name

        Parameters
        ----------
        name : str
            joint name for which it tries to find the list index

        Returns
        -------
        is_found : bool
            If the index is found it resturns True else False
        index : int
            If index is found returns the index else returns None

        Raises
        ------
        TypeError : type mismatch
            If parameter name is not type of str

        Examples
        --------
        Try to get index of a specific joint in JointSet

        >>> from xamla_motion.data_types import JointSet
        >>> joint_set1 = JointSet('joint0')
        >>> joint_set1.try_get_index_of('joint0')
        (True, 0)
        >>> joint_set1.try_get_index_of('joint1')
        (False, None)

        """
        if not isinstance(name, str):
            raise TypeError('name expected is type str')

        try:
            return True, self._names.index(name)
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

        Raises
        ------
        TypeError : type mismatch
            If parameter name is not type of str
        ValueError : value not exists
            If joint name not exists

        Examples
        --------
        Get index of a specific joint in JointSet

        >>> from xamla_motion.data_types import JointSet
        >>> joint_set1 = JointSet('joint0')
        >>> joint_set1.get_index_of('joint0')
        0
        >>> joint_set1.try_get_index_of('joint1')
        ValueError: This JointSet not contains a joint with name: joint1

        """
        if not isinstance(name, str):
            raise TypeError('name expected type is str')

        try:
            return self._names.index(name)
        except ValueError as exc:
            raise ValueError('This JointSet not contains a'
                             ' joint with name: ' + name) from exc

    def __contains__(self, names):
        """
        Checks if this JointSet contains a specific joint names

        Parameters
        ----------
        names : str or Iterable[str]
            joint names for which it checks that it is available

        Returns
        -------
        is_found : bool or Iterable of bool
            If joint names specified by names is found retruns
            True else False

        Raises
        ------
        TypeError : type mismatch
            If parameter names is not type of str

        Examples
        --------
        Get index of a specific joint in JointSet

        >>> from xamla_motion.data_types import JointSet
        >>> joint_set1 = JointSet('joint0')
        >>> 'joint0' in joint_set1
        True
        >>> 'joint1' in joint_set1
        False

        """

        if isinstance(names, str):
            return names in self._names_set
        else:
            raise TypeError('name expected type is str')

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
            return self._names[index]
        except IndexError as exc:
            raise IndexError('index out of range')
        return self._names[index]

    def __len__(self):
        return len(self._names)

    def __iter__(self):
        return self._names.__iter__()

    def __str__(self):
        return 'JointSet:\n'+'\n'.join(self._names)

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False

        if id(other) == id(self):
            return True

        if (len(self._names) != len(other.names) or
                self._names_set.difference(other.names)):
            return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)

    def __gt__(self, other):
        if not isinstance(other, self.__class__):
            return False

        if len(other) != len(self._names) and self.is_superset(other):
            return True
        else:
            return False
