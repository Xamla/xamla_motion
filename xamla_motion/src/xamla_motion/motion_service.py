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
# from future.builtins import *
from future.utils import raise_from, raise_with_traceback

import rospy
import pdb
from xamlamoveit_msgs.srv import *

from xamla_motion_exceptions import ServiceException
from data_types import *


class MotionServices(object):

    def __init__(self):

        self.__movej_action = 'moveJ_action'
        self.__query_move_group_service = ('xamlaMoveGroupServices/'
                                           'query_move_group_interface')
        self.__query_joint_states_service = ('xamlaMoveGroupServices/'
                                             'query_move_group'
                                             '_current_position')
        self.__query_forward_kinematics_service = ('xamlaMoveGroupServices/'
                                                   'query_fk')

        self.__end_effector_limits_param = ('xamlaJointJogging/'
                                            'end_effector_list')
        self.__joint_limits_param = ('robot_description_planning/'
                                     'joint_limits')

    def query_available_move_groups(self):
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

        try:
            service = rospy.ServiceProxy(
                self.__query_move_group_service,
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

    def query_available_end_effectors(self):
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
            move_groups = self.query_available_move_groups()
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

    def query_endeffector_limits(self, name):
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
        try:
            eel_param = rospy.get_param(self.__end_effector_limits_param)
        except KeyError as exc:
            raise_from(RuntimeError('end effector limit ros param: '
                                    + self.__end_effector_limits_param +
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

    def query_joint_limits(self, joint_set):
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
        if not isinstance(joint_set, JointSet):
            raise TypeError('joint_set is not of expected type JointSet')

        maxVel = [None] * len(joint_set)
        maxAcc = [None] * len(joint_set)
        minPos = [None] * len(joint_set)
        maxPos = [None] * len(joint_set)

        for i, name in enumerate(joint_set):
            prefix = self.__joint_limits_param + '/' + name + '/'

            if rospy.get_param(prefix+'has_velocity_limits'):
                maxVel[i] = rospy.get_param(prefix+'max_velocity')

            if rospy.get_param(prefix+'has_acceleration_limits'):
                maxAcc[i] = rospy.get_param(prefix+'max_acceleration')

            if rospy.get_param(prefix+'has_position_limits'):
                minPos[i] = rospy.get_param(prefix+'min_position')
                maxPos[i] = rospy.get_param(prefix+'max_position')

        return JointLimits(joint_set, maxVel, maxAcc, minPos, maxPos)

    def query_joint_states(self, joint_set):
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

        if not isinstance(joint_set, JointSet):
            raise TypeError('joint_set is not of expected type JointSet')

        try:
            service = rospy.ServiceProxy(
                self.__query_joint_states_service,
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

    def query_pose(self, move_group_name,
                   joint_positions, end_effector_link=''):
        """
        Computes the pose by applying forward kinematics

        Returns
        -------
        Pose
            Pose of the kinematic chain defined by the input parameter
        """

        move_group_name = str(move_group_name)

        return self.query_pose_many(move_group_name, joint_positions,
                                    end_effector_link)[0]

    def query_pose_many(self, move_group_name,
                        joint_path, end_effector_link=''):

        move_group_name = str(move_group_name)

        try:
            service = rospy.ServiceProxy(
                self.__query_forward_kinematics_service,
                GetFKSolution)
            response = service()
        except rospy.ServiceException as exc:
            print ('service call for query available'
                   'move groups failed, abort ')
            raise_from(ServiceException('service call for query'
                                        ' available move groups'
                                        ' failed, abort'), exc)
