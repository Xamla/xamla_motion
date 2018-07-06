# move_group.py
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

from motion_service import MotionService
from end_effector import EndEffector
from data_types import PlanParameters, TaskSpacePlanParameters

import numpy as np


class MoveGroup(object):

    def __init__(motion_service, move_group_name=None,
                 end_effector_name=None):

        if not isinstance(motion_service, MotionService)
            raise TypeError('motion_service is not of'
                            ' expected type MotionService')

        self.__m_service = motion_service

        groups = self.__m_service.query_available_move_groups()
        if not groups:
            raise RuntimeError('no move groups available')

        if end_effector_name:
            end_effector_name = str(end_effector_name)
        if move_group_name:
            move_group_name = str(move_group_name)

        if not end_effector_name and not move_group_name:
            move_group_name = groups[0].name
            end_effector_name = groups[0].end_effector_names[0]
            details = groups[0]

        elif end_effector_name and not move_group_name:
            details = next(g for g in groups
                           if any(g.end_effector_names ==
                                  end_effector_name))
            if not details:
                raise RuntimeError('it exist no move group with'
                                   ' end effector: '
                                   + end_effector_name)
            move_group_name = details.name

        elif not end_effector_name and move_group_name:
            details = next(g for g in groups
                           if g.name == move_group_name)
            if not details:
                raise RuntimeError('it exist no move group with'
                                   ' name: ' + move_group_name)
            end_effector_name = details.end_effector_name[0]
        else:
            details = next(g for g in groups
                           if (g.name == move_group_name and
                               any(g.end_effector_names ==
                                   end_effector_name)))
            if not details:
                raise RuntimeError('move group: ' + move_group_name +
                                   ' with end effector: '
                                   + end_effector_name +
                                   ' not exists')

        self.__name = move_group_name
        self.__details = details
        self.__joint_set = self.details__joint_set
        p = self.__m_service.create_plan_parameters(move_group_name,
                                                    self.__joint_set,
                                                    sample_resolution=0.05,
                                                    collision_check=False)
        self.__plan_parameters = p

        names = self.__details.end_effector_names
        link_names = self.__details.end_effector_link_names
        self.__end_effectors = {name: EndEffector(self,
                                                  name,
                                                  link_names[i])
                                for i, name in enumerate(names)}

        self.set_default_end_effector(end_effector_name)

    @property
    def name(self):
        """
        name : str (read only)
            move group name
        """
        return self.__name

    @property
    def motion_service(self):
        """
        motion_service : MotionService (read only)
            return the instance of motion service which is used
            to communicate via ros which the motion server
        """
        return self.__m_service

    @property
    def end_effector_names(self):
        """
        end_effector_names : List[str] (read only)
            List of end effectors the move groups contains
        """

        return self.__details.end_effector_names

    @property
    def selected_end_effector(self):
        """
        selected_end_effector : str (read only)
            Name of the currently selected end effector
        """

        return self.__selected_end_effector

    @property
    def joint_set(self):
        """
        joint_set : JointSet (read only)
            joint set of the move group
        """

        return self.__details.joint_set

    @property
    def collision_check(self):
        """
        collision_check : bool
            If true the trajectory planing try to plan a 
            collision free trajectory and before executing
            a trajectory a collision check is performed
        """

        return self.__plan_parameters.collision_check

    @collision_check.setter
    def collision_check(self, value):
        """
        Set collision check on or of

        Parameters
        ----------
        value : bool convertable
            activate collision check or deactivate it
        """

        If value == self.__plan_parameters.collision_check:
            return

        self.__plan_parameters.collision_check = value
        self.__task_space_plan_parameters.collision_check = value

    @property
    def sample_resolution(self):
        """
        sample_resolution : float convertable
            Trajectory point sampling frequency
            If value smaller one in seconds else in Hz
        """

        return self.__plan_parameters.sample_resolution

    @sample_resolution.setter
    def sample_resolution(self, value):
        """
        Set trajectory point sampling frequency

        Parameters
        ----------
        value : float convertable
            If value smaller one interpreted as in 
            seconds else in Hz
        """

        If value == self.__plan_parameters.sample_resolution:
            return

        self.__plan_parameters.sample_resolution = value
        self.__task_space_plan_parameters.sample_resolution = value

    @property
    def max_deviation(self):
        """
        max_deviation : float convertable
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        """

        return self.__plan_parameters.max_deviation

    @max_deviation.setter
    def max_deviation(self, value):
        """
        Set maximal devitation

        Parameters
        ----------
        value : float convertable
            maximal deviation
        """

        If value == self.__plan_parameters.max_deviation:
            return

        self.__plan_parameters.max_deviation = value
        #self.__task_space_plan_parameters.sample_resolution = value

    @property
    def ik_jump_threshold(self):
        """
        ik_jump_threshold : float convertable
            maximal allowed inverse kinematics jump threshold
        """

        return self.__task_space_plan_parameters.ik_jump_threshold

    @ik_jump_threshold.setter
    def ik_jump_threshold(self, value):
        """
        Set maximal allowed inverse kinematics jump threshold

        Parameters
        ----------
        value : float convertable
            new maximal allowed inverse kinematics jump threshold
        """

        If value == self.__task_space_plan_parameters.ik_jump_threshold:
            return

        self.__task_space_plan_parameters.ik_jump_threshold = value

    @property
    def velocity_scaling(self):
        """
        Current velocity scaling value

        velocity_scaling : float convertable
            The velocity scaling value is the value which was applied
            to the default velocities to get the current velocity limits.
            The default velocities are maximal velocities.
            The maximal velocities are provided by the motion server
            and queried from it during the initialization process.
        """
        return self.__plan_parameters.velocity_scaling

    @velocity_scaling.setter
    def velocity_scaling(self, value):

        value = float(value)

        if not np.isclose(value, self.velocity_scaling):
            self.__plan_parameters.velocity_scaling = value
            self.__task_space_plan_parameters.velocity_scaling = value

    @property
    def acceleration_scaling(self):
        """
        Current acceleration scaling value

        acceleration_scaling : float convertable
            The acceleration scaling value is the value which was applied
            to the default accelerations to get the current accelerations 
            limits. The default accelerations are maximal accelerations.
            The maximal accelerations are provided by the motion server
            and queried from it during the initialization process.
        """
        return self.__plan_parameters.acceleration_scaling

    @acceleration_scaling.setter
    def acceleration_scaling(self, value):

        value = float(value)

        if not np.isclose(value, self.acceleration_scaling):
            self.__plan_parameters.acceleration_scaling = value
            self.__task_space_plan_parameters.acceleration_scaling = value

    def set_default_end_effector(self, end_effector_name):
        """
        set one of the end effector from the list of available ones as default

        Parameters
        ----------
        end_effector_name : str convertable
            Name of the end effector which should be now the
            default end effector of this move group

        Raises
        ------
        TypeError
            If end_effector_name is not str convertable
        RuntimeError
            If the input end effector name is not available
        """

        end_effector_name = str(end_effector_name)

        try:
            self.__end_effectors[end_effector_name]
        except KeyError as exc:
            raise RuntimeError('move group' + self.__name +
                               ' has no end effector with name: ' +
                               end_effector_name) from exc

        self.__selected_end_effector = end_effector_name
        p = self.__m_service.create_task_space_plan_parameters(
            end_effector_name,
            sample_resolution=0.05,
            collision_check=False)
        self.__task_space_plan_parameters = p

    def get_end_effector(self, name=None):
        """
        Get the end effector specified by name or
        the default end effector if name is not provided

        Parameters
        ---------
        name : str convertable (optinal)
            Name of the end effector which should be returned

        Returns
        -------
        end_effector : EndEffector
            Instance of Class EndEffector with requested name

        Raises
        ------
        RuntimeError 
            If no end effector exist with the requested name
        """

        if name:
            try:
                return self.__end_effectors[name]
            except key as exc:
                raise RuntimeError('end effector with name' + name +
                                   'is not available for move group: '
                                   + self.__name) from exc
        else:
            return self.__end_effectors[self.__selected_end_effector]
