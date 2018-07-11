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


import rospy
from .xamla_motion_exceptions import *
from xamlamoveit_msgs.srv import *
from .data_types import JointValues, Pose, CartesianPath

from typing import List

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
query_cartesian_path_srv_name = '/rosvita/world_view/query_cartesianpath'
update_cartesian_path_srv_name = '/rosvita/world_view/update_cartesianpath'

add_collision_object_srv_name = '/rosvita/world_view/set_collisionobject'
get_collision_object_srv_name = '/rosvita/world_view/get_collisionobject'
query_collison_object_srv_name = '/rosvita/world_view/query_collisionobject'
update_collision_object_srv_name = '/rosvita/world_view/update_collisionobject'

remove_element_srv_name = '/rosvita/world_view/remove_element'
add_folder_srv_name = '/rosvita/world_view/add_folder'


class WorldViewClient(object):

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
        try:
            self.__add_joint_values_srv = rospy.ServiceProxy(
                add_joint_values_srv_name,
                SetJointPostureWorldView)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service: ' +
                                   add_joint_values_srv_name +
                                   ' could not be established') from exc

        try:
            self.__get_joint_values_srv = rospy.ServiceProxy(
                get_joint_values_srv_name,
                GetJointPostureWorldView)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service: ' +
                                   get_joint_values_srv_name +
                                   ' could not be established') from exc

        try:
            self.__query_joint_values_srv = rospy.ServiceProxy(
                query_joint_values_srv_name,
                QueryJointValuesWorldView)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service: ' +
                                   query_joint_values_srv_name +
                                   ' could not be established') from exc

        try:
            self.__update_joint_values_srv = rospy.ServiceProxy(
                update_joint_values_srv_name,
                UpdateJointPostureWorldView)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service: ' +
                                   update_joint_values_srv_name +
                                   ' could not be established') from exc

        # initialize all service to handle poses in world view
        try:
            self.__add_pose_srv = rospy.ServiceProxy(
                add_pose_srv_name,
                SetPoseWorldView)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service: ' +
                                   add_pose_srv_name +
                                   ' could not be established') from exc

        try:
            self.__get_pose_srv = rospy.ServiceProxy(
                get_pose_srv_name,
                GetPoseWorldView)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service: ' +
                                   get_pose_srv_name +
                                   ' could not be established') from exc

        try:
            self.__query_poses_srv = rospy.ServiceProxy(
                query_poses_srv_name,
                QueryPosesWorldView)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service: ' +
                                   query_poses_srv_name +
                                   ' could not be established') from exc

        try:
            self.__update_pose_srv = rospy.ServiceProxy(
                update_pose_srv_name,
                UpdatePoseWorldView)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service: ' +
                                   update_pose_srv_name +
                                   ' could not be established') from exc

        # initialize all service to handle cartesian path in world view
        try:
            self.__add_cartesian_path_srv = rospy.ServiceProxy(
                add_cartesian_path_srv_name,
                SetCartesianPathWorldView)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service: ' +
                                   add_cartesian_path_srv_name +
                                   ' could not be established') from exc

        try:
            self.__get_cartesian_path_srv = rospy.ServiceProxy(
                get_cartesian_path_srv_name,
                GetCartesianPathWorldView)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service: ' +
                                   get_cartesian_path_srv_name +
                                   ' could not be established') from exc

        try:
            self.__query_cartesian_paths_srv = rospy.ServiceProxy(
                query_cartesian_path_srv_name,
                QueryCartesianPathWorldView)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service: ' +
                                   query_cartesian_path_srv_name +
                                   ' could not be established') from exc

        try:
            self.__update_cartesian_path_srv = rospy.ServiceProxy(
                update_cartesian_path_srv_name,
                SetCartesianPathWorldView)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service: ' +
                                   update_cartesian_path_srv_name +
                                   ' could not be established') from exc

        # initialize all service to handle collision objects in world view
        try:
            self.__add_collision_object_srv = rospy.ServiceProxy(
                add_collision_object_srv_name,
                SetCollisionObjectWorldView)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service: ' +
                                   add_collision_object_srv_name +
                                   ' could not be established') from exc

        try:
            self.__get_collision_object_srv = rospy.ServiceProxy(
                get_collision_object_srv_name,
                GetCollisionObjectWorldView)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service: ' +
                                   get_collision_object_srv_name +
                                   ' could not be established') from exc

        try:
            self.__query_collision_objects_srv = rospy.ServiceProxy(
                query_collison_object_srv_name,
                QueryCollisionObjectWorldView)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service: ' +
                                   query_collison_object_srv_name +
                                   ' could not be established') from exc

        try:
            self.__update_collision_objects_srv = rospy.ServiceProxy(
                update_collision_object_srv_name,
                SetCollisionObjectWorldView)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service: ' +
                                   update_collision_object_srv_name +
                                   ' could not be established') from exc

    def add_joint_values(self, element_name: str, folder_path: str,
                         joint_values: JointValues, transient: bool =False):
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

        if not isinstance(folder_path, str) or not folder_path:
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
            raise ServiceException('service: ' +
                                   add_joint_values_srv_name +
                                   ' is not available') from exc

        if not response.success:
            raise ArgumentError('service call of service: '
                                + add_joint_values_srv_name +
                                ' was not successful,response with error:'
                                + response.error)

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

        if not isinstance(folder_path, str) or not folder_path:
            raise TypeError('folder_path is not of expected'
                            ' type str or is empty')

        request = GetJointPostureWorldViewRequest()
        request.element_path = folder_path + '/' + element_name

        try:
            response = self.__get_joint_values_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: ' +
                                   get_joint_values_srv_name +
                                   ' is not available') from exc

        if not response.success:
            raise ArgumentError('service call of service: '
                                + get_joint_values_srv_name +
                                ' was not successful,response with error:'
                                + response.error)

        return JointValues.from_joint_values_point_msg(response.point)

    def query_joint_values(self, folder_path: str, prefix: str ='',
                           recursive: bool = False) -> List[JointValues]:
        """
        Query all existing elements under folder_path which start with prefix

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
            raise ServiceException('service: ' +
                                   query_joint_values_srv_name +
                                   ' is not available') from exc

        if not response.success:
            raise ArgumentError('service call of service: '
                                + query_joint_values_srv_name +
                                ' was not successful,response with error:'
                                + response.error)

        if response.points:
            return [JointValues.from_joint_values_point_msg(p) for p in response.points]
        else:
            return []

    def update_joint_values(self, element_name: str, folder_path: str,
                            joint_values: JointValues, transient: bool = False):
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

        if not isinstance(folder_path, str) or not folder_path:
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
            raise ServiceException('service: ' +
                                   update_joint_values_srv_name +
                                   ' is not available') from exc

        if not response.success:
            raise ArgumentError('service call of service: '
                                + update_joint_values_srv_name +
                                ' was not successful,response with error:'
                                + response.error)

    def add_pose(self, element_name: str, folder_path: str,
                 pose: Pose, transient: bool =False):
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

        if not isinstance(folder_path, str) or not folder_path:
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
            raise ServiceException('service: ' +
                                   add_pose_srv_name +
                                   ' is not available') from exc

        if not response.success:
            raise ArgumentError('service call of service: '
                                + add_pose_srv_name +
                                ' was not successful,response with error:'
                                + response.error)

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

        if not isinstance(folder_path, str) or not folder_path:
            raise TypeError('folder_path is not of expected'
                            ' type str or is empty')

        request = GetPoseWorldViewRequest()
        request.element_path = folder_path + '/' + element_name

        try:
            response = self.__get_pose_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: ' +
                                   get_pose_srv_name +
                                   ' is not available') from exc

        if not response.success:
            raise ArgumentError('service call of service: '
                                + get_pose_srv_name +
                                ' was not successful,response with error:'
                                + response.error)

        return Pose.from_posestamped_msg(response.point)

    def query_poses(self, folder_path: str, prefix: str ='',
                    recursive: bool = False) -> List[Pose]:
        """
        Query all existing elements under folder_path which start with prefix

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
            raise ServiceException('service: ' +
                                   query_poses_srv_name +
                                   ' is not available') from exc

        if not response.success:
            raise ArgumentError('service call of service: '
                                + query_poses_srv_name +
                                ' was not successful,response with error:'
                                + response.error)

        if response.points:
            return [Pose.from_posestamped_msg(p) for p in response.points]
        else:
            return []

    def update_pose(self, element_name: str, folder_path: str,
                    pose: Pose, transient: bool = False):
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

        if not isinstance(folder_path, str) or not folder_path:
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
            raise ServiceException('service: ' +
                                   update_pose_srv_name +
                                   ' is not available') from exc

        if not response.success:
            raise ArgumentError('service call of service: '
                                + update_pose_srv_name +
                                ' was not successful,response with error:'
                                + response.error)

    def add_cartesian_path(self, element_name: str, folder_path: str,
                           cartesian_path: CartesianPath, transient: bool =False):
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

        if not isinstance(folder_path, str) or not folder_path:
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
            raise ServiceException('service: ' +
                                   add_cartesian_path_srv_name +
                                   ' is not available') from exc

        if not response.success:
            raise ArgumentError('service call of service: '
                                + add_cartesian_path_srv_name +
                                ' was not successful,response with error:'
                                + response.error)

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

        if not isinstance(folder_path, str) or not folder_path:
            raise TypeError('folder_path is not of expected'
                            ' type str or is empty')

        request = GetCartesianPathWorldView()
        request.element_path = folder_path + '/' + element_name

        try:
            response = self.__get_cartesian_path_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: ' +
                                   get_cartesian_path_srv_name +
                                   ' is not available') from exc

        if not response.success:
            raise ArgumentError('service call of service: '
                                + get_cartesian_path_srv_name +
                                ' was not successful,response with error:'
                                + response.error)

        return CartesianPath.from_cartesian_path_msg(response.path)

    def query_cartesian_paths(self, folder_path: str, prefix: str ='',
                              recursive: bool = False) -> List[CartesianPath]:
        """
        Query all existing elements under folder_path which start with prefix

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

        request = QueryCartesianPathsWorldViewRequest()
        request.prefix = prefix
        request.folder_path = folder_path
        request.recursive = recursive

        try:
            response = self.__query_cartesian_paths_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: ' +
                                   query_cartesian_paths_srv_name +
                                   ' is not available') from exc

        if not response.success:
            raise ArgumentError('service call of service: '
                                + query_cartesian_paths_srv_name +
                                ' was not successful,response with error:'
                                + response.error)

        if response.points:
            return [CartesianPath.from_cartesian_path_msg(p) for p in response.paths]
        else:
            return []

    def update_cartesian_path(self, element_name: str, folder_path: str,
                              cartesian_path: CartesianPath, transient: bool = False):
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

        if not isinstance(folder_path, str) or not folder_path:
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
        request.point = cartesian_path.to_cartesian_path_msg()
        request.transient = transient

        try:
            response = self.__update_cartesian_path_srv(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service: ' +
                                   update_cartesian_path_srv_name +
                                   ' is not available') from exc

        if not response.success:
            raise ArgumentError('service call of service: '
                                + update_cartesian_path_srv_name +
                                ' was not successful,response with error:'
                                + response.error)
