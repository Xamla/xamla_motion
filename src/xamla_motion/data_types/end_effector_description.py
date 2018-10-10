# end_effector_description.py
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

from .joint_set import JointSet


class EndEffectorDescription(object):
    """
    Class which describes an end effector 

    """

    def __init__(self, name, sub_move_group_ids, joint_set,
                 move_group_name, link_name):
        """
        Initialization of EndEffectorDescription class

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

        Returns
        ------
        EndEffectorDescription
            The created EndEffectorDescription

        Examples
        --------
        Create a instance of EndEffectorDescription

        >>> from xamla_motion.data_types import EndEffectorDescription
        >>> joint_set = JointSet('joint0')
        >>> EndEffectorDescription('tcp',[1],joint_set,['group1'],'tcp_link')
        EndEffectorDescription
        link_name = tcp_link
        name = tcp
        move_group_name = ['group1']
        joint_set = JointSet:
        joint0
        sub_move_group_ids = ['1']

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
        """
        name : str (read only)
            name of the end effector
        """
        return self.__name

    @property
    def sub_move_group_ids(self):
        """
        sub_move_group_ids : List[str] (read only)
            ids of the sub move groups
        """
        return self.__sub_move_group_ids

    @property
    def joint_set(self):
        """
        joint_set : JointSet (read only)
            JointSet which contains all joints of the base move group
        """
        return self.__joint_set

    @property
    def move_group_name(self):
        """
        move_group_name : str (read only)
            name of the move group
        """
        return self.__move_group_name

    @property
    def link_name(self):
        """
        link_name : List[str] (read only)
            names of the endeffector link
        """
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
