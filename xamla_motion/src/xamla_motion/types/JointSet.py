#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from __builtins__ import *
from functools import total_ordering


@total_ordering
class JointSet(object):
    def __init__(self, names):
        self._names: list

        if isinstance(names, str):
            self._names = [s.strip() for s in names.split(',')]
        elif names and all(isinstance(s, str) for s in names):
            self._names = names
        else:
            raise TypeError(('Wrong attribute types, only '
                             'list[str] or str with names separated '
                             'by "," are supported as attributes'))

    @classmethod
    def empty(cls):
        cls._names = []
        return cls

    def add_prefix(self, prefix):
        if not isinstance(prefix, str):
            raise TypeError('add_prefix: prefix expected type str')
        self._names = list(map(lambda x: prefix + x, self._names))

        return self

    def is_subset(self, other):
        if not isinstance(other, JointSet):
            raise TypeError('is_subset: other expected type JointSet')

        return all(name in other._names for name in self._names)

    def is_similar(self, other):
        if not isinstance(other, JointSet):
            raise TypeError('is_similar: other expected type JointSet')

        return other.count == len(self._names) and self.is_subset(other)

    def try_get_index_of(self, name):
        if not isinstance(name, str):
            raise TypeError('try_get_index_of: name expected type str')

        try:
            return True, self._names.index(name)
        except ValueError as exc:
            return False, None

    def get_index_of(self, name):
        if not isinstance(name, str):
            raise TypeError('get_index_of: name expected type str')

        try:
            return self._names.index(name)
        except ValueError as exc:
            raise ValueError('get_index_of: %s', exc)

    def count(self):
        return len(self._names)

    def this(self, index):
        if not isinstance(index, int):
            raise TypeError('this: index expected type int')

        return self._names[index]

    def contains(self, name):
        if not isinstance(index, str):
            raise TypeError('contains: name expected type str')

        is_valid, _ = self.try_get_index_of(name)

        return is_valid

    def __iter__(self):
        return self._names.__iter__()

    def __str__(self):
        return ' '.join(self._names)

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        if not isinstance(other, JointSet)
            return False

        if id(other) == id(self)
            return True

        for i, name in enumerate(self._names):
            if other._names[i] != name:
                return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        if not isinstance(other, JointSet)
            return False

        if other.count != len(self._names) and self.is_subset(other):
            return True
        else:
            return False
