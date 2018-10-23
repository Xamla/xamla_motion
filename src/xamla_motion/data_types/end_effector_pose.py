# pose.py
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

from .pose import Pose
from xamlamoveit_msgs.msg import EndEffectorPoses


class EndEffectorPose(object):
    """
    Pose defined by three dimensional translation and rotation
    """

    def __init__(self, pose: Pose, end_effector_link: str):
        """
        Initialization of the endeffector_pose class


        Parameters
        ----------
        pose : Pose
            pose of end effector
        end_effector_link : str
            Name of the link corresponding to the end effector in 
            the URDF model which is to be positioned

        Returns
        ------
        EndEffectorPose
            An instance of class EndEffectorPose

        Raises
        ------
        TypeError : type mismatch
            If tranlation vector could not converted to a numpy 
            array of shape (3,) with d type floating and the 
            quaternion is not of type Quaternion.
            If frame_id is not of type str
            If normalize_rotation is not of type bool
        ValueError
            If the quaternion initalization from 
            translation matrix went wrong
        """

        if not isinstance(pose, Pose):
            raise TypeError('pose is not of expected type Pose')

        if not isinstance(end_effector_link, str):
            raise TypeError('end_effector_link is not of expected type str')

        self.__pose = pose
        self.__end_effector_link = end_effector_link

    @property
    def pose(self):
        """
        pose : Pose (read only)
            pose of end effector
        """
        return self.__pose

    @property
    def end_effector_link(self):
        """
        end_effector_link : str (read only)
            name of the link corresponding to the end effector in 
            the URDF model which is to be positioned
        """
        return self.__end_effector_link

    def to_end_effector_pose_msg(self):
        """
        Returns a ROS EndEffectorPoses msg
        """
        ee_pose = EndEffectorPoses()
        ee_pose.poses.append(self.__pose.to_posestamped_msg())
        ee_pose.link_names.append(self.__end_effector_link)
        return ee_pose
