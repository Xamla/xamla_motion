from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from future.builtins import *
from future.utils import raise_from, raise_with_traceback

from data_types import JointSet


class EndEffectorDescription(object):
    """
    Class which describes a end effector 

    Attributes
    ----------
    name : str (read only)
        name of the end effector
    sub_move_group_ids : List[str] (read only)
        ids of the sub move groups
    joint_set : JointSet (read only)
        JointSet which contains all joints of the base move group
    move_group_name : str (read only)
        name of the move group
    link_name : List[str] (read only)
        names of the endeffector link
    """

    def __init__(self, name, sub_move_group_ids, joint_set,
                 move_group_name, link_name):
        """
        Initialization of MoveGroupDescription class

        Parameters
        ----------
        name : str convertable
            name of the end effector
        sub_move_group_ids : Iterable[str convertable]
            ids of the sub move groups
        joint_set : JointSet
            JointSet which contains all joints of the base move group
            (if provided)
        move_group_name : str convertable
            name of the move group
        link_name : str convertable
            name of the end effector link

        Raises
        ------
        TypeError : type mismatch
            If joint_set is not of type JointSet or
            ids or names are not iterable
        ValueError 
            If a input is not convertable to str

        Yields
        ------
       EndEffectorDescription
            The created EndEffectorDescription
        """

        self.__name = str(name)

        self.__sub_move_group_ids = []
        for id in sub_move_group_ids:
            self.__sub_move_group_ids.append(str(id))

        if not isinstance(joint_set, JointSet):
            raise TypeError('joint_set is not of expected type JointSet')
        self.__joint_set = joint_set

        self.__move_group_name = str(move_group_name)

        self.__link_name = str(link_name)

    @property
    def name(self):
        """end effector name (read only)"""
        return self.__name

    @property
    def sub_move_group_ids(self):
        """list of sub move group ids (read only)"""
        return self.__sub_move_group_ids

    @property
    def joint_set(self):
        """joint set (read only)"""
        return self.__joint_set

    @property
    def move_group_name(self):
        """move group name (read only)"""
        return self.__move_group_name

    @property
    def link_name(self):
        """end effector link name (read only)"""
        return self.__link_name

    def __str__(self):
        s = '\n'.join(['  '+k+' = ' + str(v)
                       for k, v in self.__dict__.items()])
        s = s.replace('_'+self.__class__.__name__+'__', '')
        print(self.__class__.__name__)
        return s

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False

        if id(other) == id(self):
            return True

        if other.name != self.__name:
            return False

        if other.sub_move_group_ids != self.__sub_move_group_ids:
            return False

        if other.joint_set != self.__joint_set:
            return False

        if other.move_group_name != self.__move_group_name:
            return False

        if other.link_name != self.__link_name:
            return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)
