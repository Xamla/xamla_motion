# gripper_client.py
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

from .data_types import MoveGripperResult, WsgCommand, WsgResult
from .motion_service import MotionService
from .xamla_motion_exceptions import ServiceException

import rospy
import asyncio


class WeissWsgGripperProperties(object):
    """
    Property class for Weiss Wsg gripper 

    This class holds the action and service names 
    for a specific Weiss Wsg gripper
    """

    def __init__(self, name, prefix='xamla/wsg_driver'):
        """
        Initialize WeissWsgProperties

        Parameters
        ----------
        name : str convertable
            Name of a specific gripper in rosvita (Configuration,
            individual wsg actuator, property Name)
        prefix : str (default = /xamla/wsg_driver/)
            Prefix of the ros service and action names
            to control a wsg gripper

        Returns
        -------
        WeissWsgProperties
            Instance of WeissWsgProperties

        Raises
        ------
        TypeError
            If name is not str convertable
            If prefix is not str convertable
        """
        prefix = str(prefix)
        prefix = prefix + '/' if not prefix.endswith('/') else prefix
        prefix = '/' + prefix if not prefix.startswith('/') else prefix
        name = str(name)
        self.__status_service_name = prefix + name + '/get_gripper_status'
        self.__set_acc_service_name = prefix + name + '/set_accelartion'
        self.__control_action_name = prefix + name + '/gripper_control'

    @property
    def status_service_name(self):
        """
        status_service_name : str
            Name of the status service for specific 
            Weiss gripper
        """
        return self.__status_service_name

    @property
    def set_acc_service_name(self):
        """
        set_acc_name : str
            Name of the set acceleration service for specific 
            Weiss gripper
        """
        return self.__set_acc_service_name

    @property
    def control_action_name(self):
        """
        control_action_name : str
            Name of the control action for specific 
            Weiss gripper
        """
        return self.__control_action_name


class WeissWsgGripper(object):

    """
    WeissWsgGripper class

    Represent a Weiss Wsg gripper and his abilities / properties

    Methods
    -------
    get_status()
        Get current status of the gripper
    acknowledge_error()
        Asynchronous acknowledge an error
    grasp(position, speed, force)
        Asynchronous action to perform a grasp
    async def homing()
        Asynchronous action to home the gripper
    move(position, speed, force, stop_on_block)
        Asynchronous action to perform a movement
    release(self, position, speed)
        Asynchronous action to release the gripper from a grasp
    stop()
        Asynchronous action to perform stop the gripper
    set_accelertation(acceleration)
        Set gripper acceleration
    """

    def __init__(self, gripper_properties, motion_service):
        """
        Initialize Weiss Wsg Gripper instance

        Parameters
        ----------
        gripper_properties : WeissWsgGripperProperties
            Properties of this gripper instance. Especially 
            the ros services names to control the gripper
        motion_service : MotionService
            Instance of MotionService to communicate with
            the motion server via ros

        Returns
        -------
        WeissWsgGripper
            Instance of WeissWsgGripper

        Raises
        ------
        TypeError
            If gripper_properties is not of type WeissWsgGripperProperties
            or if motion_service is not of type MotionService
        ServiceExeption
            If status service or set acceleration service are not 
            available            
        """

        from wsg_50_common.srv import GetGripperStatus, SetValue
        if not isinstance(gripper_properties, WeissWsgGripperProperties):
            raise TypeError('gripper_properties is not of expected'
                            ' type WeissWsgProperties')

        if not isinstance(motion_service, MotionService):
            raise TypeError('motion_service is not of expected'
                            ' type MotionService')

        self.__properties = gripper_properties
        self.__m_service = motion_service

        try:
            self.__status_service = rospy.ServiceProxy(
                self.__properties.status_service_name,
                GetGripperStatus)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service with name: ' +
                                   self.__properties.status_service_name +
                                   ' could not be established') from exc

        try:
            self.__set_acc_service = rospy.ServiceProxy(
                self.__properties.set_acc_service_name,
                SetValue)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service with name: ' +
                                   self.__properties.set_acc_service_name +
                                   ' could not be established') from exc

    @property
    def properties(self):
        """
        Properties of this Weiss wsg gripper instance
        """
        return self.__properties

    def get_status(self):
        """
        Get current status of the gripper

        Returns
        -------
        status : wsg_50_common.msg Status
            current status of the gripper

        Raises
        ------
        ServiceError
            If current service is not available or failed
        """

        try:
            response = self.__status_service()
        except rospy.ServiceException as exc:
            raise ServiceException('service with name ' +
                                   self.__properties.status_service_name +
                                   ' is not available or failed') from exc

        return response.status

    async def acknowledge_error(self):
        """
        Asynchronous acknowledge an error

        Returns
        -------
        WsgResult
            Result status of the Weiss Wsg gripper 
            after the execution of the acknowledge action

        Raises
        ------
        ServiceError
            If control action server is not available
        RuntimeError
            If action returns with unexpected result
        """
        r = await self.__m_service.wsg_gripper_command(self.__properties.control_action_name,
                                                       WsgCommand.acknowledge_error,
                                                       0.0,
                                                       0.0,
                                                       0.0,
                                                       True)

        return r

    async def grasp(self, position, speed, force):
        """
        Asynchronous action to perform a grasp

        Parameters
        ----------
        position : float convertable
            Requested position in meters
        speed : float convertable
            Requested speed in m/s
        force : float convertable
            Force which should maximally applied in Newton

        Returns
        -------
        WsgResult
            Result status of the Weiss Wsg gripper 
            after the execution of the acknowledge action

        Raises
        ------
        TypeError
            If inputs are not convertable to specified type
        ServiceError
            If control action server is not available
        RuntimeError
            If action returns with unexpected result
        """
        r = await self.__m_service.wsg_gripper_command(self.__properties.control_action_name,
                                                       WsgCommand.grasp,
                                                       float(position),
                                                       float(speed),
                                                       float(force),
                                                       True)
        return r

    async def homing(self):
        """
        Asynchronous action to home the gripper

        Returns
        -------
        WsgResult
            Result status of the Weiss Wsg gripper 
            after the execution of the acknowledge action

        Raises
        ------
        ServiceError
            If control action server is not available
        RuntimeError
            If action returns with unexpected result
        """
        r = await self.__m_service.wsg_gripper_command(self.__properties.control_action_name,
                                                       WsgCommand.homing,
                                                       0.0,
                                                       0.0,
                                                       0.0,
                                                       True)

        return r

    async def move(self, position, speed, force, stop_on_block):
        """
        Asynchronous action to perform a movement

        Parameters
        ----------
        position : float convertable
            Requested position in meters
        speed : float convertable
            Requested speed in m/s
        force : float convertable
            Force which should maximally applied in Newton
        stop_on_block : bool convertable
            If True stop if maximal force is applied

        Returns
        -------
        WsgResult
            Result status of the Weiss Wsg gripper 
            after the execution of the acknowledge action

        Raises
        ------
        TypeError
            If inputs are not convertable to specified type
        ServiceError
            If control action server is not available
        RuntimeError
            If action returns with unexpected result
        """
        r = await self.__m_service.wsg_gripper_command(self.__properties.control_action_name,
                                                       WsgCommand.move,
                                                       float(position),
                                                       float(speed),
                                                       float(force),
                                                       bool(stop_on_block))
        return r

    async def release(self, position, speed):
        """
        Asynchronous action to release the gripper from a grasp

        Parameters
        ----------
        position : float convertable
            Requested position in meters
        speed : float convertable
            Requested speed in m/s

        Returns
        -------
        WsgResult
            Result status of the Weiss Wsg gripper 
            after the execution of the acknowledge action

        Raises
        ------
        TypeError
            If inputs are not convertable to specified type
        ServiceError
            If control action server is not available
        RuntimeError
            If action returns with unexpected result
        """
        r = await self.__m_service.wsg_gripper_command(self.__properties.control_action_name,
                                                       WsgCommand.grasp,
                                                       float(position),
                                                       float(speed),
                                                       0.0,
                                                       True)

        return r

    async def stop(self):
        """
        Asynchronous action to perform stop the gripper

        Returns
        -------
        WsgResult
            Result status of the Weiss Wsg gripper 
            after the execution of the acknowledge action

        Raises
        ------
        ServiceError
            If control action server is not available
        RuntimeError
            If action returns with unexpected result
        """
        r = await self.__m_service.wsg_gripper_command(self.__properties.control_action_name,
                                                       WsgCommand.stop,
                                                       0.0,
                                                       0.0,
                                                       0.0,
                                                       True)

        return r

    def set_accelertation(self, acceleration):
        """
        Set gripper acceleration

        Parameters
        ----------
        acceleration : float convertable
            Requested acceleration

        Returns
        -------
        error : bool
            If True service call was not successful 

        Raises
        ------
        TypeError
            If inputs are not convertable to specified type
        ServiceError
            If control action server is not available
        RuntimeError
            If action returns with unexpected result
        """
        try:
            response = self.__set_acc_service(float(acceleration))
        except rospy.ServiceException as exc:
            raise ServiceException('service with name ' +
                                   self.__properties.set_acc_service_name +
                                   ' is not available or failed') from exc

        return bool(response.error)


class CommonGripperProperties(object):
    """
    Class with represents the properties of a common gripper

    This class holds the service and action names which are
    necessary to command a specific gripper
    """

    def __init__(self, name, prefix):
        """
        Initialization of CommonGripperProperties

        Parameters 
        ----------
        name : str convertable
            Name of a specific gripper in rosvita (Configuration,
            individual wsg actuator, property Name)
        prefix : str convertable
            Name of the

        Returns
        -------
        CommonGripperProperties
            Instance of CommonGripperProperties

         Raises
        ------
        TypeError
            If name is not str convertable
            If prefix is not str convertable
        """
        name = str(name)
        prefix = str(prefix)
        prefix = prefix + '/' if not prefix.endswith('/') else prefix
        prefix = '/' + prefix if not prefix.startswith('/') else prefix

        self.__command_action_name = prefix + name + '/gripper_command'

    @property
    def command_action_name(self):
        """
        command_action_name : str
            Name of the command action
        """
        return self.__command_action_name


class CommonGripper(object):
    """
    Class which represents a gripper controlled by the moveit gripper interface

    Methods
    -------
     move(position, max_effort)
        Asynchronous action to perform a movement
    """

    def __init__(self, gripper_properties, motion_service):
        """
        Initialize common gripper instance

        Parameters
        ----------
        gripper_properties : CommonGripperProperties
            Properties of this gripper instance. Especially 
            the ros services and action names to control the gripper
        motion_service : MotionService
            Instance of MotionService to communicate with
            the motion server via ros

        Returns
        -------
        CommonGripper
            Instance of CommonGripper

        Raises
        ------
        TypeError
            If gripper_properties is not of type CommonGripperProperties
            or if motion_service is not of type MotionService           
        """

        self.__properties = gripper_properties
        self.__m_service = motion_service

    async def move(self, position, max_effort):
        """
        Asynchronous action to perform a movement

        Parameters
        ----------
        position : float convertable
            Requested position in meters
        max_effort : float convertable
            Force which should maximally applied in Newton

        Returns
        -------
        MoveGripperResult
            Result status of a common gripper 
            after the execution of the move action

        Raises
        ------
        TypeError
            If inputs are not convertable to specified type
        ServiceError
            If command action server is not available
        RuntimeError
            If action returns with unexpected result
        """
        r = await self.__m_service.move_gripper(self.__properties.command_action_name,
                                                float(position),
                                                float(max_effort))
        return r
