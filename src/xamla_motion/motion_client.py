# motion_client.py
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

from .motion_service import MotionService
from .xamla_motion_exceptions import *
from .data_types import *

import numpy as np
from copy import deepcopy


class MoveGroup(object):

    """
    Class with encapsulate move functionality for a specific move group

    Methods
    -------
    set_default_end_effector
        set one of the end effector from the list of available ones as default
    get_end_effector
        Get the end effector specified by name or the default end effector
    get_current_joint_states
        Returns the current joint states of the move group joints
    get_current_joint_positions
        Returns the current joint positions of the move group joints
    plan_move_joints
        Plans a trajectory from current state to target joint positions
    plan_move_joints_collision_free
        Plans a collision free trajectory from current to target joint positions
    move_joints_collision_free
        Asynchronous plan and execute collision free joint trajectory
    move_joints
        Asynchronous plan and execute joint trajectory
    """

    def __init__(self, move_group_name=None,
                 end_effector_name=None, motion_service=None):
        """
        Initialize MoveGroup class

        The initialization process is possible with diffent settings:

        move_group_name and end_effector_name are None:
            In this case the first available move group and its
            first end effector is selected automatically. If no
            move group or end_effector is available exception is
            raised.
        move_group_name is defined and end_effector_name is none:
            In this case the initialization routine tries to find
            the move group with requested name. If it is available
            create a instance for this move group and the first
            available end effector as default end effector. Else
            an exceptions is raised.
        move_group is None and end_effector_name is defined:
            In this case the initialization routine tries to find
            the move group with requested end effector. If it is
            available create a instance for this move group and the
            requested end effector as default end effector. Else
            an exceptions is raised.
        move_group is defined and also end_effector_name is defined:
            In this case the initialization routine tries to find
            the requested move group with requested end effector.
            If it is available create a instance for this move group
            and the requested end effector as default end effector. Else
            an exceptions is raised.

        Parameters
        ----------
        move_group_name : str convertable or None
            If defined name of the move group which this
            instance should represent
        end_effector_name or None
            If defined name of the end_effector which should
            be used as default end_effector
        motion_service : MotionService or None
            An instance of MotionService which is used to
            communicate with the motion server if None a
            new instance of MotionService is created

        Raises
        ------
        TypeError
            If motion_service is not of type MotionService
            or If the other inputs are not str convertable
        ServiceError
            If necessary services to query available move
            groups and end effectors are not available
        RuntTimeError
            If requested move group or end effector not
            exist or are not available
        """

        if not motion_service:
            motion_service = MotionService()

        if not isinstance(motion_service, MotionService):
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
            try:
                end_effector_name = groups[0].end_effector_names[0]
            except IndexError:
                end_effector_name = None
            details = groups[0]

        elif end_effector_name and not move_group_name:
            try:
                details = next(g for g in groups
                               if any([end_effector_name in
                                       g.end_effector_names]))
            except StopIteration:
                raise RuntimeError('it exist no move group with'
                                   ' end effector: '
                                   + end_effector_name)
            move_group_name = details.name

        elif not end_effector_name and move_group_name:
            try:
                details = next(g for g in groups
                               if g.name == move_group_name)
            except StopIteration:
                raise RuntimeError('it exist no move group with'
                                   ' name: ' + move_group_name)

            try:
                end_effector_name = details.end_effector_name[0]
            except expression as identifier:
                end_effector_name = None

        else:
            try:
                details = next(g for g in groups
                               if any([end_effector_name in
                                       g.end_effector_names]))
            except StopIteration:
                raise RuntimeError('move group: ' + move_group_name +
                                   ' with end effector: '
                                   + end_effector_name +
                                   ' not exists')

        self.__name = move_group_name
        self.__details = details
        self.__joint_set = self.__details.joint_set
        p = self.__m_service.create_plan_parameters(move_group_name,
                                                    self.__joint_set,
                                                    sample_resolution=0.05,
                                                    collision_check=True)
        self.__plan_parameters = p

        names = self.__details.end_effector_names

        if names:
            link_names = self.__details.end_effector_link_names
            self.__end_effectors = {name: EndEffector(self,
                                                      name,
                                                      link_names[i])
                                    for i, name in enumerate(names)}

            self.set_default_end_effector(end_effector_name)
        else:
            self.__end_effectors = {}
            self.__selected_end_effector = None
            self.__task_space_plan_parameters = None

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
    def default_plan_parameters(self):
        """
        plan_parameters : PlanParameters (read only)
            Instance of PlanParameters from which the
            limits and settings are used when no specific
            user input is given
        """

        return self.__plan_parameters

    @property
    def default_task_space_plan_parameters(self):
        """
        task_space_plan_parameters : TaskSpacePlanParameters (read only)
            Instance of TaskSpacePlanParameters from which the
            limits and settings are used when no specific
            user input is given
        """

        return self.__task_space_plan_parameters

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

        if np.isclose(value, self.__plan_parameters.collision_check):
            return

        self.__plan_parameters.collision_check = value
        if self.__task_space_plan_parameters:
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

        if np.isclose(value, self.__plan_parameters.sample_resolution):
            return

        self.__plan_parameters.sample_resolution = value
        if self.__task_space_plan_parameters:
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

        if np.isclose(value, self.__plan_parameters.max_deviation):
            return

        self.__plan_parameters.max_deviation = value
        if self.__task_space_plan_parameters:
            self.__task_space_plan_parameters.max_deviation = value

    @property
    def ik_jump_threshold(self):
        """
        ik_jump_threshold : float convertable
            maximal allowed inverse kinematics jump threshold
        """
        if self.__task_space_plan_parameters:
            return self.__task_space_plan_parameters.ik_jump_threshold
        else:
            raise RuntimeError('task space plan parameters not defined'
                               ' because move group has no end effector')

    @ik_jump_threshold.setter
    def ik_jump_threshold(self, value):
        """
        Set maximal allowed inverse kinematics jump threshold

        Parameters
        ----------
        value : float convertable
            new maximal allowed inverse kinematics jump threshold
        """
        if not self.__task_space_plan_parameters:
            return

        if not np.isclose(value, self.__task_space_plan_parameters.ik_jump_threshold):
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
            if self.__task_space_plan_parameters:
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
            if self.__task_space_plan_parameters:
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
            except KeyError as exc:
                raise RuntimeError('end effector with name' + name +
                                   'is not available for move group: '
                                   + self.__name) from exc
        else:
            try:
                return self.__end_effectors[self.__selected_end_effector]
            except KeyError as exc:
                raise RuntimeError('move group: ' + self.__name +
                                   ' does not have any end effectors') from exc

    def get_current_joint_states(self):
        """
        Returns the current joint states of the move group joints

        Returns
        -------
        joint_states : JointStates
            current joint states of the move group joints

        Raises
        ------
        ServiceError
            If Service is not available or finish
            not successful
        """

        return self.__m_service.query_joint_states(self.joint_set)

    def get_current_joint_positions(self):
        """
        Returns the current joint positions of the move group joints

        Returns
        -------
        joint_positions : JointValues
            current joint position of the move group joints

        Raises
        ------
        ServiceError
            If Service is not available or finish
            not successful
        """

        return self.get_current_joint_states().positions

    def _build_plan_parameters(self, velocity_scaling=None, collision_check=None,
                               max_deviation=None, acceleration_scaling=None):
        """
        Build an instance of PlanParameters from input and default values

        All attributes with value None are ignored and instead the default
        values are used defined in default_plan_parameters are used

        Parameters
        ----------
        velocity_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint velocities
        collision_check : None or bool convertable
            If true the trajectory planing try to plan a
            collision free trajectory and before executing
            a trajectory a collision check is performed
        max_deviation : None or float convertable
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint accelerations

        Returns
        -------
        plan_parameters
            New instance of PlanParameters with requested changes
            from the default instance

        Raises
        ------
        TypeError
            If inpute values are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        """

        parameters = deepcopy(self.__plan_parameters)

        if velocity_scaling:
            parameters.velocity_scaling = velocity_scaling

        if collision_check:
            parameters.collision_check = collision_check

        if max_deviation:
            max_deviation = max_deviation

        if acceleration_scaling:
            parameters.acceleration_scaling = acceleration_scaling

        return parameters

    def _build_task_space_plan_parameters(self, velocity_scaling=None,
                                          collision_check=None,
                                          max_deviation=None,
                                          acceleration_scaling=None,
                                          end_effector_name=None):
        """
        Build an instance of TaskSpacePlanParameters from input
        and default values

        All attributes with value None are ignored and instead the default
        values are used defined in default_task_space_plan_parameters
        are used

        Parameters
        ----------
        velocity_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint velocities
        collision_check : None or bool convertable
            If true the trajectory planing try to plan a
            collision free trajectory and before executing
            a trajectory a collision check is performed
        max_deviation : None or float convertable
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint accelerations
        end_effector_name : None or str convertable
            Name of end effector which should be used

        Returns
        -------
        task_space_plan_parameters
            New instance of PlanParameters with requested changes
            from the default instance

        Raises
        ------
        TypeError
            If input values are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        """

        if not end_effector_name:
            parameters = deepcopy(self.__task_space_plan_parameters)

            if velocity_scaling:
                parameters.velocity_scaling = velocity_scaling

            if collision_check:
                parameters.collision_check = collision_check

            if max_deviation:
                max_deviation = max_deviation

            if acceleration_scaling:
                parameters.acceleration_scaling = acceleration_scaling

            return parameters

        end_effector_name = str(end_effector_name)

        try:
            self.__end_effectors[end_effector_name]
        except KeyError as exc:
            raise RuntimeError('move group' + self.__name +
                               ' has no end effector with name: ' +
                               end_effector_name) from exc

        if not velocity_scaling:
            velocity_scaling = self.__task_space_plan_parameters.velocity_scaling

        if not collision_check:
            collision_check = self.__task_space_plan_parameters.collision_check

        if not max_deviation:
            max_deviation = self.__task_space_plan_parameters.max_deviation

        if not acceleration_scaling:
            acceleration_scaling = self.__task_space_plan_parameters.acceleration_scaling

        sample_resolution = self.__task_space_plan_parameters.sample_resolution
        return self.__m_service.create_task_space_plan_parameters(
            end_effector_name,
            self.__task_space_plan_parameters.max_xyz_velocity,
            self.__task_space_plan_parameters.max_xyz_acceleration,
            self.__task_space_plan_parameters.max_angular_velocity,
            self.__task_space_plan_parameters.max_angular_acceleration,
            sample_resolution=sample_resolution,
            collision_check=collision_check,
            max_deviation=max_deviation,
            velocity_scaling=velocity_scaling,
            acceleration_scaling=acceleration_scaling)

    def plan_move_joints(self, target, velocity_scaling=None,
                         collision_check=None,
                         max_deviation=None, acceleration_scaling=None):
        """
        Plans a trajectory from current state to target joint positions

        Parameters
        ----------
        target : JointValues or JointPath
            Target joint positions or target JointPath
        velocity_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint velocities
        collision_check : None or bool convertable
            If true the trajectory planing try to plan a
            collision free trajectory and before executing
            a trajectory a collision check is performed
        max_deviation : None or float convertable
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint accelerations


        Returns
        -------
        trajectory, parameters : Tuple[JointTrajectory, PlanParameters]
            Returns a tuple where the firt argument is the planed
            trajectory and the second argument the parameters which are
            used for it


        Raises
        ------
        TypeError
            If target is not one of types JointValues, JointPath
            If all other inputs are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        ServiceError
            If underlying services from motion server are not available
            or finish not successfully
        """

        start = self.get_current_joint_positions()
        if isinstance(target, JointValues):
            joint_path = JointPath.from_start_stop_point(start, target)
        elif isinstance(target, JointPath):
            joint_path = target.prepend(start)
        else:
            raise TypeError('target is not one of expected types'
                            ' JointValues, JointPath')

        parameters = self._build_plan_parameters(velocity_scaling,
                                                 collision_check,
                                                 max_deviation,
                                                 acceleration_scaling)

        trajectory = self.__m_service.plan_move_joints(joint_path, parameters)

        return trajectory, parameters

    def plan_move_joints_collision_free(self, target, velocity_scaling=None,
                                        max_deviation=None,
                                        acceleration_scaling=None):
        """
        Plans a collision free trajectory from current to target joint positions

        Parameters
        ----------
        target : JointValues or JointPath
            Target joint positions or joint path
        velocity_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint velocities
        max_deviation : None or float convertable
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint accelerations


        Returns
        -------
        trajectory, parameters : Tuple[JointTrajectory, PlanParameters]
            Returns a tuple where the firt argument is the planed
            trajectory and the second argument the parameters which are
            used for it


        Raises
        ------
        TypeError
            If target is not one of types JointValues, JointPath
            If all other inputs are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        ServiceError
            If underlying services from motion server are not available
            or finish not successfully
        """

        start = self.get_current_joint_positions()
        if isinstance(target, JointValues):
            joint_path = JointPath.from_start_stop_point(start, target)
        elif isinstance(target, JointPath):
            joint_path = target.prepend(start)
        else:
            raise TypeError('target is not one of expected types'
                            ' JointValues, JointPath')

        parameters = self._build_plan_parameters(velocity_scaling,
                                                 False,
                                                 max_deviation,
                                                 acceleration_scaling)

        path_cf = self.__m_service.plan_collision_free_joint_path(joint_path,
                                                                  parameters)
        trajectory = self.__m_service.plan_move_joints(path_cf, parameters)

        return trajectory, parameters

    async def move_joints_collision_free(self, target, velocity_scaling=None,
                                         max_deviation=None,
                                         acceleration_scaling=None):
        """
        Asynchronous plan and execute collision free joint trajectory from joint space input

        Parameters
        ----------
        target : JointValues or JointPath
            Target joint positions or joint path
        velocity_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint velocities
        max_deviation : None or float convertable
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint accelerations

        Raises
        ------
        TypeError
            If target is not one of types JointValues, JointPath
            If all other inputs are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        ServiceError
            If underlying services from motion server are not available
            or finish not successfully
        """

        if not isinstance(target, (JointValues, JointPath)):
            raise TypeError('target is not one of expected types'
                            ' JointValues, JointPath')

        trajectory, parameters = self.plan_move_joints_collision_free(target,
                                                                      velocity_scaling,
                                                                      max_deviation,
                                                                      acceleration_scaling)

        await self.__m_service.execute_joint_trajectory(trajectory,
                                                        False)

    async def move_joints(self, target, velocity_scaling=None,
                          collision_check=None,
                          max_deviation=None,
                          acceleration_scaling=None):
        """
        Asynchronous plan and execute joint trajectory from joint space input

        Parameters
        ----------
        target : JointValues or JointPath
            Target joint positions or joint path
        velocity_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint velocities
        collision_check : None or bool convertable
            If true the trajectory planing try to plan a
            collision free trajectory and before executing
            a trajectory a collision check is performed
        max_deviation : None or float convertable
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint accelerations

        Raises
        ------
        TypeError
            If target is not one of types JointValues, JointPath
            If all other inputs are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        ServiceError
            If underlying services from motion server are not available
            or finish not successfully
        """

        if not isinstance(target, (JointValues, JointPath)):
            raise TypeError('target is not one of expected types'
                            ' JointValues, JointPath')

        trajectory, parameters = self.plan_move_joints(target,
                                                       velocity_scaling,
                                                       collision_check,
                                                       max_deviation,
                                                       acceleration_scaling)

        await self.__m_service.execute_joint_trajectory(trajectory,
                                                        parameters.collision_check)


class EndEffector(object):
    """
    Class with encapsulate move functionality for a specific end effector

    Methods
    -------
    get_current_pose
        Returns the current pose of the end effector
    move_poses
        Asynchronous plan and execute trajectory from task space input
    move_poses_collision_free
        Asynchronous plan and execute collision free trajectory from task space input
    plan_poses_linear
        Plans a trajectory with linear movements from task space input
    move_poses_linear
        Plans and executes a trajectory with linear movements from task space input
    """

    def __init__(self, move_group, end_effector_name,
                 end_effector_link_name):
        """
        Initialize EndEffector class

        Parameters
        ----------
        move_group : MoveGroup
            Instance of move group where the endeffector belongs
            to
        end_effector_name : str_convertable
            If defined name of the end_effector which should
            for which a instance of EndEffector is created
            and which is selected as default for the move
            group
        end_effector_link_name : str convertable
            name of the end effector link

        Raises
        ------
        TypeError
            If move_group is not of type MoveGroup
            or If the other inputs are not str convertable
        ServiceError
            If necessary services to query available move
            groups and end effectors are not available
        RuntTimeError
            If requested move group or end effector not
            exist or are not available
        """

        if not isinstance(move_group, MoveGroup):
            raise TypeError('move_group is not of expected'
                            ' type MoveGroup')

        self.__name = str(end_effector_name)
        self.__move_group = move_group
        self.__link_name = str(end_effector_link_name)
        self.__m_service = move_group.motion_service

    @staticmethod
    def from_end_effector_name(end_effector_name):
        """
        Creates an instance of MoveGroup and select the correct instance of EndEffector

        Parameters
        ----------
        end_effector_name : str convertable
            Name of the end_effector for which a instance of EndEffector
            should be created

        Returns
        -------
        EndEffector
            Instance of EndEffector for end effector with the name
            specified by end_effector name (instance is also hold
            by the created move_group)

        Raises
        ------
        TypeError
            If end_effector_name ist not convertable to str
        RuntimeError
            If it is not possible to create an instance of
            EndEffector because the end effector specified
            by end_effector_name is not available
        """

        end_effector_name = str(end_effector_name)

        try:
            move_group = MoveGroup(None, end_effector_name)
        except RuntimeError as exc:
            raise RuntimeError('end effector with name: ' +
                               end_effector_name +
                               ' is not available') from exc

        return move_group.get_end_effector()

    @property
    def name(self):
        """
        name : str (read only)
            end effector name
        """
        return self.__name

    @property
    def move_group(self):
        """
        move_group : MoveGroup
            Instance of MoveGroup which manages the
            move group where the end effector belongs to
        """
        return self.__move_group

    @property
    def motion_service(self):
        """
        motion_service : MotionService
            Instance of MotionService which the move group
            and all end effectors use to communicate with
            the motion server
        """
        return self.__m_service

    @property
    def link_name(self):
        """
        link_name : str
            end effector link name
        """
        return self.__link_name

    def get_current_pose(self):
        """
        Returns the current pose of the end effector

        Returns
        -------
        pose : Pose
            An instance of Pose which represents the current
            end effector pose

        Raises
        ------
        ServiceError
            If query services from motion server are not
            available or finish unsuccessfully
        """
        positions = self.__move_group.get_current_joint_positions()

        p = self.__m_service.query_pose(self.__move_group.name,
                                        positions,
                                        self.__link_name)

        return p

    async def move_poses(self, target, velocity_scaling=None,
                         collision_check=None, max_deviation=None,
                         acceleration_scaling=None, seed=None):

        """
        Asynchronous plan and execute trajectory from task space input

        Parameters
        ----------
        target : Pose or CartesianPath
            Target joint positions
        velocity_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint velocities
        collision_check : None or bool convertable
            If true the trajectory planing try to plan a
            collision free trajectory and before executing
            a trajectory a collision check is performed
        max_deviation : None or float convertable
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint accelerations

        Raises
        ------
        TypeError
            If target is not one of types Pose or CartesianPath
            If seed is defined and not of type JointValues
            If all other inputs are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        ServiceError
            If underlying services from motion server are not available
            or finish not successfully
        """

        if isinstance(target, Pose):
            target = CartesianPath.from_one_point(target)
        elif isinstance(target, CartesianPath):
            pass
        else:
            raise TypeError('target is not one of expected '
                            'types Pose or CartesianPath')

        if not seed:
            seed = self.__move_group.get_current_joint_positions()

        parameters = self.__move_group._build_plan_parameters(velocity_scaling,
                                                              collision_check,
                                                              max_deviation,
                                                              acceleration_scaling)

        ik = self.__m_service.query_inverse_kinematics_many(target,
                                                            parameters,
                                                            seed,
                                                            self.__link_name)

        if not ik.succeeded:
            raise ServiceException('inverse kinematics returns'
                                   ' with error: ' + str(ik))

        await self.__move_group.move_joints(ik.path,
                                            velocity_scaling,
                                            collision_check,
                                            max_deviation,
                                            acceleration_scaling)

    async def move_poses_collision_free(self, target,
                                        velocity_scaling=None,
                                        max_deviation=None,
                                        acceleration_scaling=None, seed=None):
        """
        Asynchronous plan and execute collision free trajectory from task space input

        Parameters
        ----------
        target : Pose or CartesianPath
            Target joint positions
        velocity_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint velocities
        max_deviation : None or float convertable
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint accelerations

        Raises
        ------
        TypeError
            If target is not one of types Pose or CartesianPath
            If seed is defined and not of type JointValues
            If all other inputs are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        ServiceError
            If underlying services from motion server are not available
            or finish not successfully
        """

        if isinstance(target, Pose):
            target = CartesianPath.from_one_point(target)
        elif isinstance(target, CartesianPath):
            pass
        else:
            raise TypeError('target is not one of expected '
                            'types Pose or CartesianPath')

        if not seed:
            seed = self.__move_group.get_current_joint_positions()

        parameters = self.__move_group._build_plan_parameters(velocity_scaling,
                                                              False,
                                                              max_deviation,
                                                              acceleration_scaling)

        ik = self.__m_service.query_inverse_kinematics_many(target,
                                                            parameters,
                                                            seed,
                                                            self.__link_name)

        if not ik.succeeded:
            raise ServiceException('inverse kinematics returns'
                                   ' with error: ' + str(ik))

        await self.__move_group.move_joints_collision_free(ik.path,
                                                           velocity_scaling,
                                                           max_deviation,
                                                           acceleration_scaling)

    def plan_poses_linear(self, target, velocity_scaling=None,
                          collision_check=None, max_deviation=None,
                          acceleration_scaling=None):
        """
        Plans a trajectory with linear movements from task space input

        Parameters
        ----------
        target : Pose or CartesianPath
            Target joint positions
        velocity_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint velocities
        collision_check : None or bool convertable
            If true the trajectory planing try to plan a
            collision free trajectory and before executing
            a trajectory a collision check is performed
        max_deviation : None or float convertable
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint accelerations

        Raises
        ------
        TypeError
            If target is not one of types Pose or CartesianPath
            If all other inputs are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        ServiceError
            If underlying services from motion server are not available
            or finish not successfully
        """

        start = self.get_current_pose()
        if isinstance(target, Pose):
            cartesian_path = CartesianPath.from_start_stop_point(start, target)
        elif isinstance(target, CartesianPath):
            cartesian_path = target.prepend(start)
        else:
            raise TypeError('target is not one of expected types'
                            ' Pose, CartesianPath')

        seed = self.__move_group.get_current_joint_positions()

        parameters = self.__move_group._build_task_space_plan_parameters(velocity_scaling,
                                                                         collision_check,
                                                                         max_deviation,
                                                                         acceleration_scaling,
                                                                         self.__name)

        trajectory = self.__m_service.plan_move_pose_linear(cartesian_path,
                                                            seed,
                                                            parameters)

        return trajectory, parameters

    async def move_poses_linear(self, target, velocity_scaling=None,
                                collision_check=None, max_deviation=None,
                                acceleration_scaling=None):

        """
        Plans and executes a trajectory with linear movements from task space input

        Parameters
        ----------
        target : Pose or CartesianPath
            Target joint positions
        velocity_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint velocities
        collision_check : None or bool convertable
            If true the trajectory planing try to plan a
            collision free trajectory and before executing
            a trajectory a collision check is performed
        max_deviation : None or float convertable
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : None or float convertable
            Scaling factor which is applied on the maximal
            possible joint accelerations

        Raises
        ------
        TypeError
            If target is not one of types Pose or CartesianPath
            If all other inputs are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        ServiceError
            If underlying services from motion server are not available
            or finish not successfully
        """

        if not isinstance(target, (Pose, CartesianPath)):
            raise TypeError('target is not one of expected types'
                            ' Pose, CartesianPath')

        trajectory, parameters = self.plan_poses_linear(target,
                                                        velocity_scaling,
                                                        collision_check,
                                                        max_deviation,
                                                        acceleration_scaling)

        await self.__m_service.execute_joint_trajectory(trajectory,
                                                        parameters.collision_check)
