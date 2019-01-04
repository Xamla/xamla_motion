# twist.py
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

import numpy as np
import geometry_msgs.msg as geometry_msgs

class Twist(object):
    """ 
    Twist contains linear velocity in m/s and Angular velocity in rad/s decribed 
    in a specific ROS TF frame

    Methods
    -------
    from_twiststamped_msg(msg)
        Initialize Pose from ROStwiststamped message
    from_twist_msg(msg, frame_id='')
        Initialize Pose from ROS pose message
    to_twiststamped_msg()
        Creates an instance of the ROS message TwistStamped from Twist
    to_twist_msg()
        Creates an instance of the ROS message Twist from Twist
    """

    def __init__(self, linear: np.array=np.array([0.0,0.0,0.0]), 
                    angular: np.array=np.array([0.0,0.0,0.0]), 
                    frame_id=""):
        """
        Initialization of the twist class

        Parameters
        ----------
        linear : convertable to numpy array of shape (3,) or None
            The linear velocity in m/s
        angular: convertable to numpy array of shape (3,) or None
            The angular velocity in rad/s
        frame_id : str (optinal default = '')
            Name of the coordinate system the pose is defined

        Returns
        ------
        Twist
            An instance of class Twist

        Raises
        ------
        TypeError
            If linear or angular vector could not be converted to a numpy 
            array of shape (3,) with d type floating  
            If frame_id is not of type str
        """
        try:
            self.__linear = np.fromiter(linear, float)
            if self.__linear.shape[0] != 3:
                raise TypeError('provided linear vector is not'
                                 ' convertable to a numpy vector of size (3,)')
        except TypeError as exc:
            raise exc

        try:
            self.__angular = np.fromiter(angular, float)
            if self.__angular.shape[0] != 3:
                raise TypeError('provided angular vector is not'
                                 ' convertable to a numpy vector of size (3,)')
        except TypeError as exc:
            raise exc
 
        if not isinstance(frame_id, str):
            raise TypeError('frame_id is not of expected type str')
        self.__frame_id = frame_id


    @classmethod
    def from_twiststamped_msg(cls, msg: geometry_msgs.TwistStamped):
        """
        Initialize Pose from ROS twiststamped message

        Parameters
        ----------
        msg : geometry_msgs.TwistStamped 
            TwistStamped message from ROS geometry_msgs

        Returns
        -------
        Twist
            Instance of Twist generated from TwistStamped message

        Raises
        ------
        TypeError
            If msg is not of type geometry_msgs.TwistStamped
        """
        if not isinstance(msg, geometry_msgs.TwistStamped):
            raise TypeError('msg is not of expected type TwistStamped')

        frame_id = msg.header.frame_id

        if not frame_id:
            frame_id = ""

        return cls.from_twist_msg(msg.twist, frame_id)

    @classmethod
    def from_twist_msg(cls, msg: geometry_msgs.Twist, frame_id = ""):
        """
        Initialize Pose from ROS twist message

        Parameters
        ----------
        msg : geometry_msgs.Twist
            Twist message  from ROS geometry_msgs

        Returns
        -------
        Twist
            Instance of Twist generated from Twist message

        Raises
        ------
        TypeError
            If msg is not of type geometry_msgs.Twist
        """
        if not isinstance(msg, geometry_msgs.Twist):
            raise TypeError('msg is not of expected type Twist')

        twist_msg = msg.twist
        twist = cls.from_twist_msg

        linear = np.fromiter([msg.linear.x,
                                msg.linear.y,
                                msg.linear.z],
                                float)

        angular =  np.fromiter([msg.angular.x,
                                msg.angular.y,
                                msg.angular.z],
                                float)

        frame_id = msg.header.frame_id

        if not isinstance(frame_id, str):
            raise TypeError('frame_id is not of expected type str')
        return cls(linear, angular, frame_id)

    def to_twiststamped_msg(self) -> geometry_msgs.TwistStamped:
        """
        Creates an instance of the ROS message TwistStamped

        Returns
        ------
        geometry_msgs.TwistStamped
            Instance of ROS message TwistStamped (seq and time are not set)
            docs.ros.org/kinetic/api/geometry_msgs/html/msg/TwistStamped.html
        """
        twist_stamped = geometry_msgs.TwistStamped()
        twist_stamped.header.frame_id = self.__frame_id
        twist_stamped.twist = self.to_twist_msg()

        return twist_stamped

    def to_twist_msg(self) -> geometry_msgs.Twist:
        """
        Creates an instance of the ROS message Twist

        Returns
        ------
        geometry_msgs.Twist
            Instance of ROS message Twist
            docs.ros.org/kinetic/api/geometry_msgs/html/msg/Twist.html
        """

        twist = geometry_msgs.Twist()

        twist.linear.x = self.__linear[0]
        twist.linear.y = self.__linear[1]
        twist.linear.z = self.__linear[2]

        twist.angular.x = self.__angular[0]
        twist.angular.y = self.__angular[1]
        twist.angular.z = self.__angular[2]

        return twist

    @property
    def frame_id(self) -> str:
        """
        frame_id : str (readonly)
            Id of the coordinate system / frame
        """
        return self.__frame_id

    @property
    def linear(self) -> np.array:
        """
        translation : np.array((3,) dtype=floating) (readonly)
            numpy row array of size 3 which describes the linear velocity
        """
        return self.__linear

    @property
    def angular(self) -> np.array:
        """
        translation : np.array((3,) dtype=floating) (readonly)
            numpy row array of size 3 which describes the angular velocity
        """
        return self.__angular

    def __str__(self):
        axes = tuple(['x', 'y', 'z'])

        linear_str = '\n'.join(['linear.'+axes[i] + ' : '
                                     + str(value) for i, value
                                     in enumerate(self.__linear)])
        angular_str = '\n'.join(['angular.'+axes[i] + ' : ' + str(value)
                                    for i, value
                                    in enumerate(self.__angular)])

        return ('Twist:\n' + linear_str + '\n' + angular_str +
                '\nframe_id : ' + self.__frame_id)

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        r_tol = 1.0e-13
        a_tol = 1.0e-14

        if not isinstance(other, self.__class__):
            return False

        if id(other) == id(self):
            return True

        if not np.allclose(self.__linear, other.linear,
                           rtol=r_tol, atol=a_tol):
            return False
        if not np.allclose(self.__angular, other.angular,
                           rtol=r_tol, atol=a_tol):
            return False
        return True

    def __ne__(self, other):
        return not self.__eq__(other)

  