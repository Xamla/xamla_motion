# gripper.py
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

from data_types import MoveGripperResult, WsgCommand, WsgResult
from motion_service import MotionService
from xamla_motion_exceptions import ServiceException

import rospy
import asyncio


class WeissWsgGripperProperties(object):
    """
    Property class for Weiss Wsg gripper which holds
    the action and service names for a specific gripper
    """

    def __init__(self, name, prefix='/xamla/wsg_driver/'):
        """
        Initialize WeissWsgProperties

        Parameters
        ----------
        name : str convertable
            Name of a specific gripper in rosvita (Configuration,
            individual wsg actuator, property Name )

        Returns
        -------
        WeissWsgProperties
            Instance of WeissWsgProperties

        Raises
        ------
        TypeError
            If name is not str convertable
        """

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

    def __init__(self, gripper_properties, motion_service):
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

    async def get_status(self):
        """
        Asynchron get current status of the gripper

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
        r = await self.__m_service.wsg_gripper_command(self.__properties.control_action_name,
                                                       WsgCommand.acknowledge_error,
                                                       0.0,
                                                       0.0,
                                                       0.0,
                                                       True)

        return r

    async def grasp(self, position, speed, force):
        r = await self.__m_service.wsg_gripper_command(self.__properties.control_action_name,
                                                       WsgCommand.grasp,
                                                       float(position),
                                                       float(speed),
                                                       float(force),
                                                       True)
        return r

    async def homing(self):
        r = await self.__m_service.wsg_gripper_command(self.__properties.control_action_name,
                                                       WsgCommand.homing,
                                                       0.0,
                                                       0.0,
                                                       0.0,
                                                       True)

        return r

    async def move(self, position, speed, force, stop_one_block):
        r = await self.__m_service.wsg_gripper_command(self.__properties.control_action_name,
                                                       WsgCommand.move,
                                                       float(position),
                                                       float(speed),
                                                       float(force),
                                                       bool(stop_one_block))

    async def release(self, position, speed):
        r = await self.__m_service.wsg_gripper_command(self.__properties.control_action_name,
                                                       WsgCommand.grasp,
                                                       float(position),
                                                       float(speed),
                                                       0.0,
                                                       True)

        return r

    async def stop(self):
        r = await self.__m_service.wsg_gripper_command(self.__properties.control_action_name,
                                                       WsgCommand.stop,
                                                       0.0,
                                                       0.0,
                                                       0.0,
                                                       True)

        return r

    async def set_accelertation(self, acceleration):
        try:
            response = self.__set_acc_service(acceleration)
        except rospy.ServiceException as exc:
            raise ServiceException('service with name ' +
                                   self.__properties.set_acc_service_name +
                                   ' is not available or failed') from exc

        return response.status
