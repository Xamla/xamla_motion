# world_view_client.py
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


import pathlib
from typing import List, Dict

import moveit_msgs.msg as moveit_msgs
import rospy

from xamlamoveit_msgs.srv import (CreateFolderWorldView,
                                  CreateFolderWorldViewRequest,
                                  RemoveElementWorldView,
                                  RemoveElementWorldViewRequest)

from xamlamoveit_msgs.srv import (SetJointPostureWorldView,
                                  SetJointPostureWorldViewRequest,
                                  GetJointPostureWorldView,
                                  GetJointPostureWorldViewRequest,
                                  UpdateJointPostureWorldView,
                                  UpdateJointPostureWorldViewRequest,
                                  QueryJointValuesWorldView,
                                  QueryJointValuesWorldViewRequest)

from xamlamoveit_msgs.srv import (SetPoseWorldView,
                                  SetPoseWorldViewRequest,
                                  GetPoseWorldView,
                                  GetPoseWorldViewRequest,
                                  UpdatePoseWorldView,
                                  UpdatePoseWorldViewRequest,
                                  QueryPosesWorldView,
                                  QueryPosesWorldViewRequest)

from xamlamoveit_msgs.srv import (SetCartesianPathWorldView,
                                  SetCartesianPathWorldViewRequest,
                                  GetCartesianPathWorldView,
                                  GetCartesianPathWorldViewRequest,
                                  QueryCartesianPathWorldView,
                                  QueryCartesianPathWorldViewRequest)

from xamlamoveit_msgs.srv import (SetCollisionObjectWorldView,
                                  SetCollisionObjectWorldViewRequest,
                                  GetCollisionObjectWorldView,
                                  GetCollisionObjectWorldViewRequest,
                                  QueryCollisionObjectWorldView,
                                  QueryCollisionObjectWorldViewRequest)

from .data_types import CartesianPath, CollisionObject, JointValues, Pose
from .xamla_motion_exceptions import ServiceException, ArgumentError

add_joint_values_srv_name = '/rosvita/world_view/add_joint_posture'
get_joint_values_srv_name = '/rosvita/world_view/get_joint_posture'
query_joint_values_srv_name = '/rosvita/world_view/query_joint_postures'
update_joint_values_srv_name = '/rosvita/world_view/update_joint_posture'

add_pose_srv_name = '/rosvita/world_view/add_pose'
get_pose_srv_name = '/rosvita/world_view/get_pose'
query_poses_srv_name = '/rosvita/world_view/query_poses'
update_pose_srv_name = '/rosvita/world_view/update_pose'

add_cartesian_path_srv_name = '/rosvita/world_view/set_cartesianpath'
get_cartesian_path_srv_name = '/rosvita/world_view/get_cartesianpath'
query_cartesian_paths_srv_name = '/rosvita/world_view/query_cartesianpath'
update_cartesian_path_srv_name = '/rosvita/world_view/update_cartesianpath'

add_collision_object_srv_name = '/rosvita/world_view/set_collisionobject'
get_collision_object_srv_name = '/rosvita/world_view/get_collisionobject'
query_collision_objects_srv_name = '/rosvita/world_view/query_collisionobject'
update_collision_object_srv_name = '/rosvita/world_view/update_collisionobject'

remove_element_srv_name = '/rosvita/world_view/remove_element'
add_folder_srv_name = '/rosvita/world_view/add_folder'


class WorldViewClient(object):

    """
    Client to interact and manipulate the Rosvita WorldView

    With help of this client class it is possible to add, get,
    query and remove different kinds of elements to or from
    the Rosvita world view.

    Methods
    -------
    add_joint_values
        Add / store joint values element in world view tree
    get_joint_values
        Get joint values element from world view tree
    query_joint_values
        Query all joint values under folder_path which start with prefix
    update_joint_values
        Update an already existing joint values element in world view tree
    add_pose
        Add / store pose element in world view tree
    get_pose
        Get pose element from world view tree
    query_poses
        Query all existing poses under folder_path which start with prefix
    update_pose
        Update an already existing pose element in world view tree
    add_cartesian_path
        Add / store cartesian path element in world view tree
    get_cartesian_path
        Get cartesian path element from world view tree
    query_cartesian_pahts
        Query all existing castesian paths under folder_path which start with prefix
    update_cartesian_path
        Update an already existing collision object element in world view tree
    add_collision_object
        Add / store collision object element in world view tree
    get_collision_object
        Get collision object element from world view tree
    query_cartesian_pahts
        Query all existing castesian paths under folder_path which start with prefix
    update_collision_object
        Update an already existing collision object element in world view tree
    """

    def __init__(self):
        """
        Initialize WorldViewClient

        Returns
        -------
        Instance of WorldViewClient

        Raises
        ------
        ServiceError
            If connection to world view services
            could not be established
        """

        # initialize all service to handle joint values in world view
        self.__add_joint_values_srv = rospy.ServiceProxy(
            add_joint_values_srv_name,
            SetJointPostureWorldView)

        self.__get_joint_values_srv = rospy.ServiceProxy(
            get_joint_values_srv_name,
            GetJointPostureWorldView)

        self.__query_joint_values_srv = rospy.ServiceProxy(
            query_joint_values_srv_name,
            QueryJointValuesWorldView)

        self.__update_joint_values_srv = rospy.ServiceProxy(
            update_joint_values_srv_name,
            UpdateJointPostureWorldView)

        # initialize all service to handle poses in world view
        self.__add_pose_srv = rospy.ServiceProxy(
            add_pose_srv_name,
            SetPoseWorldView)

        self.__get_pose_srv = rospy.ServiceProxy(
            get_pose_srv_name,
            GetPoseWorldView)

        self.__query_poses_srv = rospy.ServiceProxy(
            query_poses_srv_name,
            QueryPosesWorldView)

        self.__update_pose_srv = rospy.ServiceProxy(
            update_pose_srv_name,
            UpdatePoseWorldView)

        # initialize all service to handle cartesian path in world view
        self.__add_cartesian_path_srv = rospy.ServiceProxy(
            add_cartesian_path_srv_name,
            SetCartesianPathWorldView)

        self.__get_cartesian_path_srv = rospy.ServiceProxy(
            get_cartesian_path_srv_name,
            GetCartesianPathWorldView)

        self.__query_cartesian_paths_srv = rospy.ServiceProxy(
            query_cartesian_paths_srv_name,
            QueryCartesianPathWorldView)

        self.__update_cartesian_path_srv = rospy.ServiceProxy(
            update_cartesian_path_srv_name,
            SetCartesianPathWorldView)

        # initialize all service to handle collision objects in world view
        self.__add_collision_object_srv = rospy.ServiceProxy(
            add_collision_object_srv_name,
            SetCollisionObjectWorldView)

        self.__get_collision_object_srv = rospy.ServiceProxy(
            get_collision_object_srv_name,
            GetCollisionObjectWorldView)

        self.__query_collision_objects_srv = rospy.ServiceProxy(
            query_collision_objects_srv_name,
            QueryCollisionObjectWorldView)

        self.__update_collision_objects_srv = rospy.ServiceProxy(
            update_collision_object_srv_name,
            SetCollisionObjectWorldView)

        # add folder service
        self.__add_folder_srv = rospy.ServiceProxy(
            add_folder_srv_name,
            CreateFolderWorldView)

        # remove element service
        self.__remove_element_srv = rospy.ServiceProxy(
            remove_element_srv_name,
            RemoveElementWorldView)

    def add_joint_values(self, element_name: str, folder_path: str,
                         joint_values: JointValues, transient: bool=False):
        """
        Add / store joint values element in world view tree

        Parameters
        ----------
        element_name : str
            Name of the new added joint values element
        folder_path : str
            Path in world view tree where the new joint values element
            should be added / stored
        joint_values : JointValues
            Instance of joint values which is added / stored
        transient : bool
            If True the added joint values are only valid for this session
            and will not be saved in a rosvita project context

        Raises
        ------
        ServiceError
            If necessary service is not available
        ArgumentError
            If it was not possible to add joint values
            due to wrong path or element already exists

        """

        if not isinstance(element_name, str) or not element_name:
            raise TypeError('element_name is not of expected'
                            ' type str or is empty')

        if not isinstance(folder_path, str):
            raise TypeError('folder_path is not of expected'
                            ' type str or is empty')

        if not isinstance(joint_values, JointValues):
            raise TypeError('joint_values is not of expected type JointValues')

        if not isinstance(transient, bool):
            raise TypeError('transient is not of expected type bool')

        request = SetJointPostureWorldViewRequest()
        request.display_name = element_name
        request.element_path = folder_path
        request.point = joint_values.to_joint_values_point_msg()
        request.transient = transient

        try:
            response = self.__add_joint_values_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: {} is not available'
                                   ''.format(add_joint_values_srv_name)
                                   ) from exc

        if not response.success:
            raise ArgumentError('service call of service: {}'
                                ' was not successful,response with'
                                ' error: {}'.format(add_joint_values_srv_name,
                                                    response.error))

    def get_joint_values(self, element_name: str,
                         folder_path: str) -> JointValues:
        """
        Get joint values element from world view tree

        Parameters
        ----------
        element_name : str
            Name of the requested element
        folder_path : str
            Full path to folder where the requested element is located
            from world view tree root

        Returns
        -------
        joint_values : JointValues
            Instance of JointValues with
            the values which are defined in the
            requested joint values element

        Raises
        ------
        ServiceError
            If necessary service is not available
        ArgumentError
            If it was not possible to get joint values
            due to wrong path or element not exists
        """

        if not isinstance(element_name, str) or not element_name:
            raise TypeError('element_name is not of '
                            'expected type str or is empty')

        if not isinstance(folder_path, str):
            raise TypeError('folder_path is not of expected'
                            ' type str or is empty')

        request = GetJointPostureWorldViewRequest()
        request.element_path = '/'.join([folder_path, element_name])

        try:
            response = self.__get_joint_values_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: {} is not available'
                                   ''.format(get_joint_values_srv_name)
                                   ) from exc

        if not response.success:
            raise ArgumentError('service call of service: {}'
                                ' was not successful,response with'
                                ' error: {}'.format(get_joint_values_srv_name,
                                                    response.error))

        return JointValues.from_joint_values_point_msg(response.point)

    def query_joint_values(self, folder_path: str, prefix: str='',
                           recursive: bool=False) -> Dict[pathlib.PurePath, JointValues]:
        """
        Query all existing joint values elements under folder_path which start with prefix

        Parameters
        ----------
        folder_path : str
            Path to folder in which the search should be performed
        prefix : str (default empty string)
            Query elements which start with this prefix
        recursive : bool (default False)
            If True query from folder path recursively

        Returns
        -------
        result : List[JointValues]
            List of JointValues in alphanumeric order see world view

        Raises
        ------
        ServiceError
            If necessary service is not available
        ArgumentError
            If it was not possible to query joint values
            due to not existing path
        """

        if not isinstance(folder_path, str):
            raise TypeError(
                'folder_path is not of expected type str or is empty')
        if not isinstance(prefix, str):
            raise TypeError('path is not of expected type str')
        if not isinstance(recursive, bool):
            raise TypeError('recursive is not of expected type bool')

        request = QueryJointValuesWorldViewRequest()
        request.prefix = prefix
        request.folder_path = folder_path
        request.recursive = recursive

        try:
            response = self.__query_joint_values_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: {} is not available'
                                   ''.format(query_joint_values_srv_name)
                                   ) from exc

        if not response.success:
            raise ArgumentError('service call of service: {}'
                                ' was not successful,response with'
                                ' error: {}'.format(query_joint_values_srv_name,
                                                    response.error))

        if response.points:
            result = {}
            for n, e, p in zip(response.names,
                               response.element_paths,
                               response.points):

                path = pathlib.PurePath(e) / n
                result[path] = JointValues.from_joint_values_point_msg(p)
            return result
        else:
            return {}

    def update_joint_values(self, element_name: str, folder_path: str,
                            joint_values: JointValues, transient: bool=False):
        """
        Update already existing joint values element in world view tree

        Parameters
        ----------
        element_name : str
            Name of the element which should be updated
        folder_path : str
            Path in world view tree where the joint values element
            is located
        joint_values : JointValues
            Instance of joint values which contains the new values
        transient : bool
            If True the updated joint values are only valid for this session
            and will not be saved in a rosvita project context
        Raises
        ------
        ServiceError
            If necessary service is not available
        ArgumentError
            If it was not possible to update joint values
            due to element not exists
        """

        if not isinstance(element_name, str) or not element_name:
            raise TypeError('element_name is not of expected'
                            ' type str or is empty')

        if not isinstance(folder_path, str):
            raise TypeError('folder_path is not of expected'
                            ' type str or is empty')

        if not isinstance(joint_values, JointValues):
            raise TypeError('joint_values is not of expected type JointValues')

        if not isinstance(transient, bool):
            raise TypeError('transient is not of expected type bool')

        request = UpdateJointPostureWorldViewRequest()
        request.display_name = element_name
        request.element_path = folder_path
        request.point = joint_values.to_joint_values_point_msg()
        request.transient = transient

        try:
            response = self.__update_joint_values_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: {} is not available'
                                   ''.format(update_joint_values_srv_name)
                                   ) from exc

        if not response.success:
            raise ArgumentError('service call of service: {}'
                                ' was not successful,response with'
                                ' error: {}'.format(update_joint_values_srv_name,
                                                    response.error))

    def add_pose(self, element_name: str, folder_path: str,
                 pose: Pose, transient: bool=False):
        """
        Add / store pose element in world view tree

        Parameters
        ----------
        element_name : str
            Name of the new added pose element
        folder_path : str
            Path in world view tree where the new pose element
            should be added / stored
        pose : Pose
            Instance of pose which is added / stored
        transient : bool
            If True the added pose is only valid for this session
            and will not be saved in a rosvita project context

        Raises
        ------
        ServiceError
            If necessary service is not available
        ArgumentError
            If it was not possible to add pose
            due to wrong path or element already exists

        """

        if not isinstance(element_name, str) or not element_name:
            raise TypeError('element_name is not of expected'
                            ' type str or is empty')

        if not isinstance(folder_path, str):
            raise TypeError('folder_path is not of expected'
                            ' type str or is empty')

        if not isinstance(pose, Pose):
            raise TypeError('pose is not of expected type Pose')

        if not isinstance(transient, bool):
            raise TypeError('transient is not of expected type bool')

        request = SetPoseWorldViewRequest()
        request.display_name = element_name
        request.element_path = folder_path
        request.point = pose.to_posestamped_msg()
        request.transient = transient

        try:
            response = self.__add_pose_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: {} is not available'
                                   ''.format(add_pose_srv_name)
                                   ) from exc

        if not response.success:
            raise ArgumentError('service call of service: {}'
                                ' was not successful,response with'
                                ' error: {}'.format(add_pose_srv_name,
                                                    response.error))

    def get_pose(self, element_name: str,
                 folder_path: str) -> Pose:
        """
        Get pose element from world view tree

        Parameters
        ----------
        element_name : str
            Name of the requested element
        folder_path : str
            Full path to folder where the requested element is located
            from world view tree root

        Returns
        -------
        pose : Pose
            Instance of Pose with
            the values which are defined in the
            requested pose element

        Raises
        ------
        ServiceError
            If necessary service is not available
        ArgumentError
            If it was not possible to get pose
            due to wrong path or element not exists
        """

        if not isinstance(element_name, str) or not element_name:
            raise TypeError('element_name is not of '
                            'expected type str or is empty')

        if not isinstance(folder_path, str):
            raise TypeError('folder_path is not of expected'
                            ' type str or is empty')

        request = GetPoseWorldViewRequest()
        request.element_path = '/'.join([folder_path, element_name])

        try:
            response = self.__get_pose_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: {} is not available'
                                   ''.format(get_pose_srv_name)
                                   ) from exc

        if not response.success:
            raise ArgumentError('service call of service: {}'
                                ' was not successful,response with'
                                ' error: {}'.format(get_pose_srv_name,
                                                    response.error))

        return Pose.from_posestamped_msg(response.point)

    def query_poses(self, folder_path: str, prefix: str='',
                    recursive: bool=False) -> List[Pose]:
        """
        Query all existing pose elements under folder_path which start with prefix

        Parameters
        ----------
        folder_path : str
            Path to folder in which the search should be performed
        prefix : str (default empty string)
            Query elements which start with this prefix
        recursive : bool (default False)
            If True query from folder path recursively

        Returns
        -------
        result : List[Pose]
            List of Pose in alphanumeric order see world view

        Raises
        ------
        ServiceError
            If necessary service is not available
        ArgumentError
            If it was not possible to query pose
            due to not existing path
        """

        if not isinstance(folder_path, str):
            raise TypeError(
                'folder_path is not of expected type str or is empty')
        if not isinstance(prefix, str):
            raise TypeError('path is not of expected type str')
        if not isinstance(recursive, bool):
            raise TypeError('recursive is not of expected type bool')

        request = QueryPosesWorldViewRequest()
        request.prefix = prefix
        request.folder_path = folder_path
        request.recursive = recursive

        try:
            response = self.__query_poses_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: {} is not available'
                                   ''.format(query_poses_srv_name)
                                   ) from exc

        if not response.success:
            raise ArgumentError('service call of service: {}'
                                ' was not successful,response with'
                                ' error: {}'.format(query_poses_srv_name,
                                                    response.error))

        if response.points:
            result = {}
            for n, e, p in zip(response.names,
                               response.element_paths,
                               response.points):

                path = pathlib.PurePath(e) / n
                result[path] = Pose.from_posestamped_msg(p)
            return result
        else:
            return {}

    def update_pose(self, element_name: str, folder_path: str,
                    pose: Pose, transient: bool=False):
        """
        Update already existing pose element in world view tree

        Parameters
        ----------
        element_name : str
            Name of the element which should be updated
        folder_path : str
            Path in world view tree where the pose element
            is located
        pose : Pose
            Instance of pose which contains the new values
        transient : bool
            If True the updated pose is only valid for this session
            and will not be saved in a rosvita project context

        Raises
        ------
        ServiceError
            If necessary service is not available
        ArgumentError
            If it was not possible to update pose
            due to element not exists
        """

        if not isinstance(element_name, str) or not element_name:
            raise TypeError('element_name is not of expected'
                            ' type str or is empty')

        if not isinstance(folder_path, str):
            raise TypeError('folder_path is not of expected'
                            ' type str or is empty')

        if not isinstance(pose, Pose):
            raise TypeError('pose is not of expected type Pose')

        if not isinstance(transient, bool):
            raise TypeError('transient is not of expected type bool')

        request = UpdatePoseWorldViewRequest()
        request.display_name = element_name
        request.element_path = folder_path
        request.point = pose.to_posestamped_msg()
        request.transient = transient

        try:
            response = self.__update_pose_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: {} is not available'
                                   ''.format(update_pose_srv_name)
                                   ) from exc

        if not response.success:
            raise ArgumentError('service call of service: {}'
                                ' was not successful,response with'
                                ' error: {}'.format(update_pose_srv_name,
                                                    response.error))

    def add_cartesian_path(self, element_name: str, folder_path: str,
                           cartesian_path: CartesianPath, transient: bool=False):
        """
        Add / store cartesian path element in world view tree

        Parameters
        ----------
        element_name : str
            Name of the new added cartesian path element
        folder_path : str
            Path in world view tree where the new cartesian path element
            should be added / stored
        cartesian_path : CartesianPath
            Instance of cartesian path which is added / stored
        transient : bool
            If True the added cartesian path is only valid for this session
            and will not be saved in a rosvita project context

        Raises
        ------
        ServiceError
            If necessary service is not available
        ArgumentError
            If it was not possible to add cartesian path
            due to wrong path or element already exists

        """

        if not isinstance(element_name, str) or not element_name:
            raise TypeError('element_name is not of expected'
                            ' type str or is empty')

        if not isinstance(folder_path, str):
            raise TypeError('folder_path is not of expected'
                            ' type str or is empty')

        if not isinstance(cartesian_path, CartesianPath):
            raise TypeError(
                'cartesian path is not of expected type CartesianPath')

        if not isinstance(transient, bool):
            raise TypeError('transient is not of expected type bool')

        request = SetCartesianPathWorldViewRequest()
        request.display_name = element_name
        request.element_path = folder_path
        request.path = cartesian_path.to_cartesian_path_msg()
        request.transient = transient

        try:
            response = self.__add_cartesian_path_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: {} is not available'
                                   ''.format(add_cartesian_path_srv_name)
                                   ) from exc

        if not response.success:
            raise ArgumentError('service call of service: {}'
                                ' was not successful,response with'
                                ' error: {}'.format(add_cartesian_path_srv_name,
                                                    response.error))

    def get_cartesian_path(self, element_name: str,
                           folder_path: str) -> CartesianPath:
        """
        Get cartesian path element from world view tree

        Parameters
        ----------
        element_name : str
            Name of the requested element
        folder_path : str
            Full path to folder where the requested element is located
            from world view tree root

        Returns
        -------
        cartesian_path : CartesianPath
            Instance of CartesianPath with
            the values which are defined in the
            requested cartesian path element

        Raises
        ------
        ServiceError
            If necessary service is not available
        ArgumentError
            If it was not possible to get cartesian path
            due to wrong path or element not exists
        """

        if not isinstance(element_name, str) or not element_name:
            raise TypeError('element_name is not of '
                            'expected type str or is empty')

        if not isinstance(folder_path, str):
            raise TypeError('folder_path is not of expected'
                            ' type str or is empty')

        request = GetCartesianPathWorldViewRequest()
        request.element_path = '/'.join([folder_path, element_name])

        try:
            response = self.__get_cartesian_path_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: {} is not available'
                                   ''.format(get_cartesian_path_srv_name)
                                   ) from exc

        if not response.success:
            raise ArgumentError('service call of service: {}'
                                ' was not successful,response with'
                                ' error: {}'.format(get_cartesian_path_srv_name,
                                                    response.error))

        return CartesianPath.from_cartesian_path_msg(response.path)

    def query_cartesian_paths(self, folder_path: str, prefix: str='',
                              recursive: bool=False) -> List[CartesianPath]:
        """
        Query all existing cartesian path elements under folder_path which start with prefix

        Parameters
        ----------
        folder_path : str
            Path to folder in which the search should be performed
        prefix : str (default empty string)
            Query elements which start with this prefix
        recursive : bool (default False)
            If True query from folder path recursively

        Returns
        -------
        result : List[CartesianPath]
            List of CartesianPath in alphanumeric order see world view

        Raises
        ------
        ServiceError
            If necessary service is not available
        ArgumentError
            If it was not possible to query cartesian path
            due to not existing path
        """

        if not isinstance(folder_path, str):
            raise TypeError(
                'folder_path is not of expected type str or is empty')
        if not isinstance(prefix, str):
            raise TypeError('path is not of expected type str')
        if not isinstance(recursive, bool):
            raise TypeError('recursive is not of expected type bool')

        request = QueryCartesianPathWorldViewRequest()
        request.prefix = prefix
        request.folder_path = folder_path
        request.recursive = recursive

        try:
            response = self.__query_cartesian_paths_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: {} is not available'
                                   ''.format(query_cartesian_paths_srv_name)
                                   ) from exc

        if not response.success:
            raise ArgumentError('service call of service: {}'
                                ' was not successful,response with'
                                ' error: {}'.format(query_cartesian_paths_srv_name,
                                                    response.error))

        if response.paths:
            result = {}
            for n, e, p in zip(response.names,
                               response.element_paths,
                               response.points):

                path = pathlib.PurePath(e) / n
                result[path] = CartesianPath.from_cartesian_path_msg(p)
            return result
        else:
            return {}

    def update_cartesian_path(self, element_name: str, folder_path: str,
                              cartesian_path: CartesianPath, transient: bool=False):
        """
        Update already existing cartesian path element in world view tree

        Parameters
        ----------
        element_name : str
            Name of the element which should be updated
        folder_path : str
            Path in world view tree where the cartesian path element
            is located
        cartesian_path : CartesianPath
            Instance of cartesian path which contains the new values
        transient : bool
            If True the updated cartesian path is only valid for this session
            and will not be saved in a rosvita project context

        Raises
        ------
        ServiceError
            If necessary service is not available
        ArgumentError
            If it was not possible to update cartesian path
            due to element not exists
        """

        if not isinstance(element_name, str) or not element_name:
            raise TypeError('element_name is not of expected'
                            ' type str or is empty')

        if not isinstance(folder_path, str):
            raise TypeError('folder_path is not of expected'
                            ' type str or is empty')

        if not isinstance(cartesian_path, CartesianPath):
            raise TypeError(
                'cartesian path is not of expected type CartesianPath')

        if not isinstance(transient, bool):
            raise TypeError('transient is not of expected type bool')

        request = SetCartesianPathWorldViewRequest()
        request.display_name = element_name
        request.element_path = folder_path
        request.path = cartesian_path.to_cartesian_path_msg()
        request.transient = transient

        try:
            response = self.__update_cartesian_path_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: {} is not available'
                                   ''.format(update_cartesian_path_srv_name)
                                   ) from exc

        if not response.success:
            raise ArgumentError('service call of service: {}'
                                ' was not successful,response with'
                                ' error: {}'.format(update_cartesian_path_srv_name,
                                                    response.error))

    def add_collision_object(self, element_name: str, folder_path: str,
                             collision_object: CollisionObject, transient: bool=False):
        """
        Add / store collision object element in world view tree

        Parameters
        ----------
        element_name : str
            Name of the new added collision object element
        folder_path : str
            Path in world view tree where the new collision object element
            should be added / stored
        collision_object : CollisionObject
            Instance of collision object which is added / stored
        transient : bool
            If True the added collision object is only valid for this session
            and will not be saved in a rosvita project context

        Raises
        ------
        ServiceError
            If necessary service is not available
        ArgumentError
            If it was not possible to add collision object
            due to wrong path or element already exists

        """

        if not isinstance(element_name, str) or not element_name:
            raise TypeError('element_name is not of expected'
                            ' type str or is empty')

        if not isinstance(folder_path, str):
            raise TypeError('folder_path is not of expected'
                            ' type str or is empty')

        if not isinstance(collision_object, CollisionObject):
            raise TypeError(
                'collision object is not of expected type CollisionObject')

        if not isinstance(transient, bool):
            raise TypeError('transient is not of expected type bool')

        request = SetCollisionObjectWorldViewRequest()
        request.collision_object = collision_object.to_collision_object_msg(
            moveit_msgs.CollisionObject.ADD)
        request.display_name = element_name
        request.element_path = folder_path
        request.transient = transient

        try:
            response = self.__add_collision_object_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: {} is not available'
                                   ''.format(add_collision_object_srv_name)
                                   ) from exc

        if not response.success:
            raise ArgumentError('service call of service: {}'
                                ' was not successful,response with'
                                ' error: {}'.format(add_collision_object_srv_name,
                                                    response.error))

    def get_collision_object(self, element_name: str,
                             folder_path: str) -> CollisionObject:
        """
        Get collision object element from world view tree

        Parameters
        ----------
        element_name : str
            Name of the requested element
        folder_path : str
            Full path to folder where the requested element is located
            from world view tree root

        Returns
        -------
        collision_object : CollisionObject
            Instance of CollisionObject with
            the values which are defined in the
            requested collision object element

        Raises
        ------
        ServiceError
            If necessary service is not available
        ArgumentError
            If it was not possible to get collision object
            due to wrong path or element not exists
        """

        if not isinstance(element_name, str) or not element_name:
            raise TypeError('element_name is not of '
                            'expected type str or is empty')

        if not isinstance(folder_path, str):
            raise TypeError('folder_path is not of expected'
                            ' type str or is empty')

        request = GetCollisionObjectWorldViewRequest()
        request.element_path = '/'.join([folder_path, element_name])

        try:
            response = self.__get_collision_object_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: {} is not available'
                                   ''.format(get_collision_object_srv_name)
                                   ) from exc

        if not response.success:
            raise ArgumentError('service call of service: {}'
                                ' was not successful,response with'
                                ' error: {}'.format(get_collision_object_srv_name,
                                                    response.error))

        return CollisionObject.from_collision_object_msg(response.collision_object)

    def query_collision_objects(self, folder_path: str, prefix: str='',
                                recursive: bool=False) -> List[CollisionObject]:
        """
        Query all existing collision objet elements under folder_path which start with prefix

        Parameters
        ----------
        folder_path : str
            Path to folder in which the search should be performed
        prefix : str (default empty string)
            Query elements which start with this prefix
        recursive : bool (default False)
            If True query from folder path recursively

        Returns
        -------
        result : List[CollisionObject]
            List of CollisionObject in alphanumeric order see world view

        Raises
        ------
        ServiceError
            If necessary service is not available
        ArgumentError
            If it was not possible to query collision object
            due to not existing path
        """

        if not isinstance(folder_path, str):
            raise TypeError(
                'folder_path is not of expected type str or is empty')
        if not isinstance(prefix, str):
            raise TypeError('path is not of expected type str')
        if not isinstance(recursive, bool):
            raise TypeError('recursive is not of expected type bool')

        request = QueryCollisionObjectWorldViewRequest()
        request.prefix = prefix
        request.folder_path = folder_path
        request.recursive = recursive

        try:
            response = self.__query_collision_objects_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: {} is not available'
                                   ''.format(query_collision_objects_srv_name)
                                   ) from exc

        if not response.success:
            raise ArgumentError('service call of service: {}'
                                ' was not successful,response with'
                                ' error: {}'.format(query_collision_objects_srv_name,
                                                    response.error))

        if response.collision_objects:
            result = {}
            for n, e, p in zip(response.names,
                               response.element_paths,
                               response.points):

                path = pathlib.PurePath(e) / n
                result[path] = CollisionObject.from_collision_object_msg(p)
            return result
        else:
            return {}

    def update_collision_object(self, element_name: str, folder_path: str,
                                collision_object: CollisionObject, transient: bool=False):
        """
        Update already existing collision object element in world view tree

        Parameters
        ----------
        element_name : str
            Name of the element which should be updated
        folder_path : str
            Path in world view tree where the collision object element
            is located
        collision_object : CollisionObject
            Instance of collision object which contains the new values
        transient : bool
            If True the updated collision object is only valid for this session
            and will not be saved in a rosvita project context

        Raises
        ------
        ServiceError
            If necessary service is not available
        ArgumentError
            If it was not possible to update collision object
            due to element not exists
        """

        if not isinstance(element_name, str) or not element_name:
            raise TypeError('element_name is not of expected'
                            ' type str or is empty')

        if not isinstance(folder_path, str):
            raise TypeError('folder_path is not of expected'
                            ' type str or is empty')

        if not isinstance(collision_object, CollisionObject):
            raise TypeError(
                'collision object is not of expected type CollisionObject')

        if not isinstance(transient, bool):
            raise TypeError('transient is not of expected type bool')

        request = SetCollisionObjectWorldViewRequest()
        request.display_name = element_name
        request.element_path = folder_path
        request.collision_object = collision_object.to_collision_object_msg(
            moveit_msgs.CollisionObject.ADD)
        request.transient = transient

        try:
            response = self.__update_collision_objects_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: {} is not available'
                                   ''.format(update_collision_object_srv_name)
                                   ) from exc

        if not response.success:
            raise ArgumentError('service call of service: {}'
                                ' was not successful,response with'
                                ' error: {}'.format(update_collision_object_srv_name,
                                                    response.error))

    def add_folder(self, folder_name, folder_path):
        """
        Add a folder to world view tree

        Parameters
        ----------
        folder_name : str
            Name of the folder which should be added
        folder_path : str
            Path in world view tree where the folder should be added

        Raises
        ------
        ServiceError
            If necessary service is not available
        ArgumentError
            If it was not possible to add folder
            input path not exists
        """

        if not isinstance(folder_name, str) or not folder_name:
            raise TypeError('folder_name is not of expected'
                            ' type str or is empty')

        if not isinstance(folder_path, str):
            raise TypeError('folder_path is not of expected'
                            ' type str or is empty')

        request = CreateFolderWorldViewRequest()
        request.folder_name = folder_name
        request.folder_path = folder_path

        try:
            response = self.__add_folder_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: {} is not available'
                                   ''.format(add_folder_srv_name)
                                   ) from exc

        if not response.success:
            raise ArgumentError('service call of service: {}'
                                ' was not successful,response with'
                                ' error: {}'.format(add_folder_srv_name,
                                                    response.error))

    def remove_element(self, element_name, folder_path):
        """
        Remove existing elements from world view tree

        Parameters
        ----------
        element_name : str
            Name of the element which should be removed
        folder_path : str
            Path in world view tree where the element
            is located

        Raises
        ------
        ServiceError
            If necessary service is not available
        ArgumentError
            If it was not possible to update collision object
            due to element not exists
        """

        if not isinstance(element_name, str) or not element_name:
            raise TypeError('element_name is not of expected'
                            ' type str or is empty')

        if not isinstance(folder_path, str):
            raise TypeError('folder_path is not of expected'
                            ' type str or is empty')

        request = RemoveElementWorldViewRequest()
        request.element_path = '/'.join([folder_path, element_name])

        try:
            response = self.__remove_element_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: {} is not available'
                                   ''.format(remove_element_srv_name)
                                   ) from exc

        if not response.success:
            raise ArgumentError('service call of service: {}'
                                ' was not successful,response with'
                                ' error: {}'.format(remove_element_srv_name,
                                                    response.error))
