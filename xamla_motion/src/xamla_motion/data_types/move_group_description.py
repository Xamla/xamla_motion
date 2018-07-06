# move_group_description.py
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

from data_types import JointSet


class MoveGroupDescription(object):

    """
    Class which describes a robot or a subset of it 
    by move groups, end effectors and joints
    """

    def __init__(self, name, sub_move_group_ids, joint_set,
                 end_effector_names, end_effector_link_names):
        """
        Initialization of MoveGroupDescription class

        Parameters
        ----------
        name : str convertable
            name of the base move group
        sub_move_group_ids : Iterable[str convertable]
            ids of the sub move groups
        joint_set : JointSet
            JointSet which contains all joints of the base move group
            (if provided)
        end_effector_names : Iterable[str convertable]
            names of the end effectors
        end_effector_link_names : Iterable[str convertable]
            names of the end effector links

        Raises
        ------
        TypeError : type mismatch
            If joint_set is not of type JointSet or
            ids or names are not iterable
        ValueError 
            If a input is not convertable to str

        Returns
        ------
        MoveGroupDescription
            The created MoveGroupDescription
        """

        self.__name = str(name)

        self.__sub_move_group_ids = []
        for id in sub_move_group_ids:
            self.__sub_move_group_ids.append(str(id))

        if not isinstance(joint_set, JointSet):
            raise TypeError('joint_set is not of expected type JointSet')
        self.__joint_set = joint_set

        self.__end_effector_names = []
        for end_effector_name in end_effector_names:
            self.__end_effector_names.append(str(end_effector_name))

        self.__end_effector_link_names = []
        for end_effector_link_name in end_effector_link_names:
            self.__end_effector_link_names.append(str(end_effector_link_name))

    @property
    def name(self):
        """
        name : str (read only)
            name of the base move group
        """
        return self.__name

    @property
    def sub_move_group_ids(self):
        """
        sub_move_group_ids: List[str](read only)
            ids of the sub move groups
        """
        return self.__sub_move_group_ids

    @property
    def joint_set(self):
        """
        joint_set: JointSet(read only)
            JointSet which contains all joints of the base move group
        """
        return self.__joint_set

    @property
    def end_effector_names(self):
        """
        end_effector_names: List[str](read only)
            names of the end effectors
        """
        return self.__end_effector_names

    @property
    def end_effector_link_names(self):
        """
        end_effector_link_names: List[str](read only)
            names of the end effector links
        """
        return self.__end_effector_link_names

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

        if other.end_effector_names != self.__end_effector_names:
            return False

        if other.end_effector_link_names != self.__end_effector_link_names:
            return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)
