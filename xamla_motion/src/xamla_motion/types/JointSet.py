#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from future.builtins import *
from functools import total_ordering


@total_ordering
class JointSet(object):
    """
    Class which manages a list of joint names

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
    count()
        Returns number of joint names
    contains(name)
        Checks if this JointSet contains a specific joint name
    """

    def __init__(self, names):
        """
        Initialization of the JointSet class

        Parameters
        ----------
        names : str or list of str 
            The Joint set class is initializable in different ways.
            -by a string which only contains one joint name
            -by a string which contains multiple joints names
             separated by a comma as delimiter
            -by a list of strings where each string represents a
             joint name

        Yields
        ------
            An instance of class JointSet

        Raises
        ------
        TypeError : type mismatch
            If input parameter names is not of type str or list of str
        """
        self.__names = tuple()

        if isinstance(names, str):
            self.__names = tuple(s.strip() for s in names.split(','))
        elif names and all(isinstance(s, str) for s in names):
            self.__names = tuple(names)
        else:
            raise TypeError(('Wrong attribute types, only '
                             'list[str] or str with names separated '
                             'by "," are supported as attributes'))

    @staticmethod
    def empty():
        """
        Creates a empty JointSet

        Yields
        ------
        JointSet
            The created empty JointSet
        """
        joint_set = JointSet('')
        joint_set.__names = []
        return joint_set

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

        Yields
        ------
        JointSet
            The created JointSet with added prefix to joint names    

        """
        if not isinstance(prefix, str):
            raise TypeError('prefix expected type is str')
        names = list(map(lambda x: prefix + x, self.__names))
        return JointSet(names)

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
        if not isinstance(other, JointSet):
            raise TypeError('other expected type is JointSet')

        return all(name in other.__names for name in self.__names)

    def is_similar(self, other):
        """
        Checks if it contains the same joint names as in JointSet other

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
        if not isinstance(other, JointSet):
            raise TypeError('other expected type is JointSet')

        return other.count == len(self.__names) and self.is_subset(other)

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

    def count(self):
        """
        Returns the number of joint names

        Returns
        -------
        count : int
            Number of joint names which are managed by this JointSet
        """
        return len(self.__names)

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

    def __iter__(self):
        return self.__names.__iter__()

    def __str__(self):
        return ' '.join(self.__names)

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        if not isinstance(other, JointSet):
            return False

        if id(other) == id(self):
            return True

        for i, name in enumerate(self.__names):
            if other.__names[i] != name:
                return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        if not isinstance(other, JointSet):
            return False

        if other.count != len(self.__names) and self.is_subset(other):
            return True
        else:
            return False
