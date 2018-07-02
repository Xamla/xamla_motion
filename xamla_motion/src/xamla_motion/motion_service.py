# motion_service.py
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
                        print_function)
from future.builtins import map
from future.utils import raise_from, raise_with_traceback

import rospy
import pdb
from xamlamoveit_msgs.srv import *
from moveit_msgs.msg import MoveItErrorCodes

from xamla_motion_exceptions import ServiceException, ArgumentError
from data_types import *


class MotionServices(object):

    __instance = None

    def __new__(cls, val):
        if MotionServices.__instance is None:
            MotionServices.__instance = object.__new__(cls)
        MotionServices.__instance.velocity_scaling = 1.0
        MotionServices.__instance.acceleration_scaling = 1.0
        return MotionServices.__instance

    def __init__(self):

        self.__movej_action = 'moveJ_action'

        self.__joint_limits_param = ('robot_description_planning/'
                                     'joint_limits')

    @staticmethod
    def query_available_move_groups():
        """
        Query all currently available move groups

        To query the move groups the ros service with the string
        defined in query_move_group_serivce is called

        Returns
        -------
        groups : List[MoveGroupDescription]
            Returns a list with instances of MoveGroupDescription.
            For further details please take a look into the documentation
            of MoveGroupDescription

        Raises
        ------
        xamla_motion.ServiceNotAvailableException
            If Service not exist or is not callable
        TypeError
            If the service response joint_names are not
            of an iterable type
        """

        query_move_group_serivce = ('xamlaMoveGroupServices/'
                                    'query_move_group_interface')

        try:
            service = rospy.ServiceProxy(
                query_move_group_service,
                QueryMoveGroupInterfaces)
            response = service()
        except rospy.ServiceException as exc:
            raise_from(ServiceException('service call for query'
                                        ' available move groups failed,'
                                        ' abort '), exc)

        groups = list()
        for g in response.move_group_interfaces:
            if len(g.joint_names) == 0:
                joint_set = JointSet.empty()
            else:
                joint_set = JointSet(g.joint_names)
            groups.append(MoveGroupDescription(g.name, g.sub_move_group_ids,
                                               joint_set, g.end_effector_names,
                                               g.end_effector_link_names))
        return groups

    @staticmethod
    def query_available_end_effectors():
        """
        Query all currently available end effectors

        To query the available end effectors the ros service with the string
        defined in _query_move_group_service is called an only the relevant
        information about the endeffector is filtered out

        Returns
        -------
        end_effectors : Dict[str, EndEffectorDescription]
            Returns a dictionary key is the endeffector name,
            value is an instance of EndEffectorDescription

        Raises
        ------
        xamla_motion.ServiceExeption
            If Service not exist or is not callable
        TypeError
            If the service response joint_names are not
            of an iterable type
        """
        try:
            move_groups = MotionServices.query_available_move_groups()
        except (ServiceException, TypeError) as exc:
            raise exc

        end_effectors = {}

        for group in move_groups:
            for i, end_effector in enumerate(group.end_effector_names):
                if end_effector not in end_effectors:
                    d = EndEffectorDescription(end_effector,
                                               group.sub_move_group_ids,
                                               group.joint_set, group.name,
                                               group.end_effector_link_names[i])

                    end_effectors[end_effector] = d

        return end_effectors

    @staticmethod
    def query_endeffector_limits(name):
        """
        Query end effector limits from ros param

        To query the end effector limits the ros param definied in
        end_effector_limits_param is read out

        Parameters
        ----------
        name : str convertable
            Name of the end effector for which the
            limits are queried

        Returns
        -------
        EndEffectorLimits
            Returns a instance of EndEffectorLimits

        Raises
        ------
        RuntimeError
            If the the ros parameter or
            the requested end effector name
            no exists
        KeyError
            If not all necessary limits exists
        """

        end_effector_limits_param = ('xamlaJointJogging/'
                                     'end_effector_list')

        try:
            eel_param = rospy.get_param(end_effector_limits_param)
        except KeyError as exc:
            raise_from(RuntimeError('end effector limit ros param: '
                                    + end_effector_limits_param +
                                    ' not exists'), exc)
        name = str(name)

        for limits in eel_param:

            try:
                end_effector_name = limits['name']
            except KeyError as exc:
                continue

            if name == end_effector_name:
                max_xyz_vel = limits['taskspace_xyz_max_vel']
                max_xyz_acc = limits['taskspace_xyz_max_acc']
                max_angular_vel = limits['taskspace_angular_max_vel']
                max_angular_acc = limits['taskspace_angular_max_acc']
                return EndEffectorLimits(max_xyz_vel,
                                         max_xyz_acc,
                                         max_angular_vel,
                                         max_angular_acc)
            else:
                continue

        raise RuntimeError('Requested end effector name not exists')

    @staticmethod
    def query_joint_limits(joint_set):
        """
        Query end joint limits from ros param

        To query the joint limits the ros param definied in
        joint_limits_param + joint name + limit name is read out

        Parameters
        ----------
        join_set : JointSet
            Set of joints for which the
            limits are queried

        Returns
        -------
        JointLimits
            Returns a instance of JointLimits
            invalid limits are of type numpy.nan

        Raises
        ------
        TypeError : type mismatch
            If joint_set is not of expected type JointSet
        KeyError
            If ros params not exists
        """
        joint_limits_param = ('robot_description_planning/'
                              'joint_limits')

        if not isinstance(joint_set, JointSet):
            raise TypeError('joint_set is not of expected type JointSet')

        maxVel = [None] * len(joint_set)
        maxAcc = [None] * len(joint_set)
        minPos = [None] * len(joint_set)
        maxPos = [None] * len(joint_set)

        for i, name in enumerate(joint_set):
            prefix = joint_limits_param + '/' + name + '/'

            if rospy.get_param(prefix+'has_velocity_limits'):
                maxVel[i] = rospy.get_param(prefix+'max_velocity')

            if rospy.get_param(prefix+'has_acceleration_limits'):
                maxAcc[i] = rospy.get_param(prefix+'max_acceleration')

            if rospy.get_param(prefix+'has_position_limits'):
                minPos[i] = rospy.get_param(prefix+'min_position')
                maxPos[i] = rospy.get_param(prefix+'max_position')

        return JointLimits(joint_set, maxVel, maxAcc, minPos, maxPos)

    @staticmethod
    def query_joint_states(joint_set):
        """
        Query joint states by calling the providing ros service

        To query the joint states the ros service with the name
        defined in quary_joint_states_service is called

        Parameters
        ----------
        joint_set : JointSet
            Set of joints for which the joint states are queried

        Returns
        -------
        JointStates
            Returns a instance of JointStates which contains the joint
            states of all joints defined in joint_set

        Raises
        ------
        TypeError : type mismatch
            If joint_set is not of expected type JointSet
        xamla_motion.ServiceException
            If ros service not exists or is not callable
        """
        query_joint_states_service = ('xamlaMoveGroupServices/'
                                      'query_move_group'
                                      '_current_position')

        if not isinstance(joint_set, JointSet):
            raise TypeError('joint_set is not of expected type JointSet')

        try:
            service = rospy.ServiceProxy(
                query_joint_states_service,
                GetCurrentJointState)
            response = service(joint_set.names).current_joint_position
        except rospy.ServiceException as exc:
            print ('service call for query current'
                   'joint states failed, abort ')
            raise_from(ServiceException('service call for query'
                                        ' current joint states '
                                        'failed, abort'), exc)

        r_joint_set = JointSet(response.name)
        positions = JointValues(r_joint_set,
                                response.position)

        if not response.velocity:
            velocities = None
        else:
            velocities = JointValues(r_joint_set,
                                     response.velocity)

        if not response.effort:
            efforts = None
        else:
            efforts = JointValues(r_joint_set,
                                  response.effort)

        return JointStates(positions, velocities, efforts)

    @staticmethod
    def query_pose(move_group_name,
                   joint_positions, end_effector_link=''):
        """
        Computes the pose by applying forward kinematics

        Parameters
        ----------
        move_group_name : str convertable
            name of the move group from which the pose
            is queried
        joint_positions : JointValues
            joint values from which the pose is calculated
        end_effector_link : str convertable (optional)
            end effector link is necessary if end effector
            is not part of the move group but pose should
            be computed for the end effector

        Returns
        -------
        Pose
            Pose of the kinematic chain defined by the input parameter

        Raises
        ------
        TypeError
            If joint_positions is not of type JointValues
        """

        if not isinstance(joint_positions, JointValues):
            raise TypeError('joint_positions is not of'
                            ' expected type JointValues')

        joint_path = JointPath.from_one_point(joint_positions)
        return MotionServices.query_pose_many(move_group_name, joint_path,
                                              end_effector_link)[0]

    @staticmethod
    def query_pose_many(move_group_name,
                        joint_path, end_effector_link=''):
        """
        Query the poses from joint path points by applying forward kinematics

        Parameters
        ----------
        move_group_name : str convertable
            name of the move group from which the poses are queried
        joint_path : JointPath
            joint path from which the poses are calculated
        end_effector_link : str convertable (optional)
            end effector link is necessary if end effector
            is not part of the move group but pose should
            be computed for the end effector

        Returns
        -------
        List[Pose]
            List of Poses

        Raises
        ------
        TypeError
            If joint_path is not of type JointPath
        """

        query_forward_kinematics_service = ('xamlaMoveGroupServices/'
                                            'query_fk')

        move_group_name = str(move_group_name)
        end_effector_link = str(end_effector_link)

        if not isinstance(joint_path, JointPath):
            raise TypeError('joint_path is not of expected type JointPath')

        try:
            service = rospy.ServiceProxy(
                query_forward_kinematics_service,
                GetFKSolution)
            response = service(move_group_name,
                               end_effector_link,
                               joint_path.joint_set,
                               [p.to_joint_path_point_message()
                                for p in joint_path])
        except rospy.ServiceException as exc:
            print ('service call for query forward kinematics'
                   ' failed, abort ')
            raise_from(ServiceException('service call for query'
                                        ' forward kinematics'
                                        ' failed, abort'), exc)

        if (response.error_codes == None or response.error_msgs == None
                or len(response.error_codes) != len(response.error_msgs)):
            raise ServiceException('service call for query forward'
                                   'kinematics returns with '
                                   'invalid response')

        errors = None
        for i, error in enumerate(response.error_codes):
            if error.val != MoveItErrorCodes.SUCCESS:
                raise ServiceException('service call for query forward'
                                       ' kinematics was not'
                                       ' successful for point: ' + str(i) +
                                       'service name:' +
                                       query_forward_kinematics_service +
                                       ' error code: ' +
                                       str(response.error_code.val))

        return list(map(lambda x: Pose.from_posestamped_msg(x),
                        response.solutions))

    @staticmethod
    def _query_moveit_joint_path(move_group_name, joint_path):

        query_joint_path_service = ('xamlaPlanningServices/'
                                    'query_joint_path')

        if not isinstance(joint_path, JointPath):
            raise TypeError('joint_path is not of expected type JointPath')

        try:
            service = rospy.ServiceProxy(
                query_joint_path_service,
                GetMoveItJointPath)
            response = service(move_group_name,
                               joint_path.joint_set,
                               [p.to_joint_path_point_message()
                                for p in joint_path])
        except rospy.ServiceException as exc:
            print ('service call for query joint path'
                   ' failed, abort ')
            raise_from(ServiceException('service call for query'
                                        ' joint path'
                                        ' failed, abort'), exc)

        return response

    @staticmethod
    def query_collision_free_joint_path(move_group_name, joint_path):
        """
        Query a collision free joint path from a user defined joint path

        Parameters
        ----------
        move_group_name : str convertable
            name of the move group for which the collision free joint
            is required
        joint_path : JointPath
            joint path which may contains collisions

        Results
        -------
        JointPath
            New planned JointPath without collisions

        Raises
        ------
        TypeError : type mismatch
            If joint_path is not of expectef type JointPath
            or it move_group_name is not convertable to str

        ServiceException
            If query joint path service is not callable or
            is not successful
        """

        query_joint_path_service = ('xamlaPlanningServices/'
                                    'query_joint_path')

        if not isinstance(joint_path, JointPath):
            raise TypeError('joint_path is not of expected type JointPath')

        move_group_name = str(move_group_name)

        response = MotionServices._query_moveit_joint_path(move_group_name,
                                                           joint_path)

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise ServiceException('service call for query collision free'
                                   ' joint path was not successful. '
                                   'service name:' +
                                   query_joint_path_service +
                                   ' error code: ' +
                                   str(response.error_code.val))

        return JointPath(joint_path.joints, [JointValues(joint_path.joints, p)
                                             for p in respones.path])

    @staticmethod
    def query_cartesian_path(cartesian_path, number_of_steps=50):
        """
        Query a complete cartesian path

        from a cartesian path with
        key points e.g. only start stop point

        Parameters
        ----------
        cartesian_path : CartesianPath
            Documentation
        number_of_steps : int convertable
            Documentation

        Returns
        -------
        CartesianPath
            New planned cartesian path

        Raises
        ------
        TypeError
            If cartesian path is not of expected type cartesian path
            or if number_of_steps is not convertable to int
        """

        query_cartesian_path_service = ('xamlaPlanningServices/'
                                        'query_cartesian_path')

        if not isinstance(cartesian_path, CartesianPath):
            raise TypeError('cartesian_path is not of expected type'
                            ' CartesianPath')

        number_of_steps = int(number_of_steps)

        try:
            service = rospy.ServiceProxy(
                query_cartesian_path_service,
                GetLinearCartesianPath)
            response = service([p.to_posestamped_msg()
                                for p in cartesian_path],
                               number_of_steps)
        except rospy.ServiceException as exc:
            print ('service call for query cartesian path'
                   ' failed, abort ')
            raise_from(ServiceException('service call for query'
                                        ' cartesian path'
                                        ' failed, abort'), exc)

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise ServiceException('service call for query cartesian'
                                   ' path was not successful. '
                                   'service name:' +
                                   query_cartesian_path_service +
                                   ' error code: ' +
                                   str(response.error_code.val))

        return CartesianPath([Pose.from_posestamped_msg(p)
                              for p in response.path])

    @staticmethod
    def query_joint_trajectory(joint_path, max_velocity, max_acceleration,
                               max_devitation, delta_t):

        query_joint_trajectory_service = ('xamlaPlanningService/'
                                          'query_joint_trajectory')

        if not isinstance(joint_path, JointPath):
            raise TypeError('joint_path is not of expected type JointPath')

        max_velocity = [float(v) for v in max_velocity]
        max_acceleration = [float(v) for v in max_acceleration]
        max_devitation = float(max_devitation)
        delta_t = float(delta_t)

        delta_t = 1 / delta_t if delta_t > 1.0 else delta_t

        try:
            service = rospy.ServiceProxy(
                query_joint_trajectory_service,
                GetOptimJointTrajectory)
            response = service(joint_path.joints,
                               [p.to_joint_path_point_message()
                                for p in joint_path],
                               max_velocity,
                               max_acceleration,
                               max_devitation,
                               delta_t)
        except rospy.ServiceException as exc:
            print ('service call for query joint trajectory'
                   ' failed, abort ')
            raise_from(ServiceException('service call for query'
                                        ' joint trajectory'
                                        ' failed, abort'), exc)

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise ServiceException('service call for query joint'
                                   ' trajectory was not successful. '
                                   'service name:' +
                                   query_joint_trajectory_service +
                                   ' error code: ' +
                                   str(response.error_code.val))

        result_joint_set = JointSet(response.solution.joint_names)
