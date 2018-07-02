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

#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function)  # , unicode_literals)
#from future.builtins import *
from future.utils import raise_from, raise_with_traceback

from pyquaternion import Quaternion
from geometry_msgs.msg import PoseStamped
import numpy as np


class Pose(object):
    """
    Pose defined by three dimensional translation and rotation

    The translation is representated by a three dimensional
    row vector. The rotation is represented by a
    quaternion. The pose itself is idenified by the frame_id
    attribute

    Methods
    -------
    from_transformation_matrix
        Creates an instance of Pose from a transformation matrix
    from_posestamped_msg
        Initialize Pose from ROS posestamped message
    normalize_rotation
        Creates an instance of Pose with normalized quaternion
    is_rotation_normalized
        Return True if quaternion is normalized
    rotation_matrix
        Returns the roation matrix in homogenous coordinates (4x4 numpy array)
    transformation_matrix
        Returns the transformation matrix in homogenous coordinates (4x4 numpy array)
    to_posestamped_msg
        Creates an instance of the ROS message PoseStamped from Pose
    """

    def __init__(self, translation, rotation, frame_id='',
                 normalize_rotation=False):
        """
        Initialization of the pose class

        For the internal quaternion representation and also to initialize the
        the class the library pyquaternion is used :
        http://kieranwynn.github.io/pyquaternion/

        Parameters
        ----------
        translation : convertable to numpy array of shape (3,)
            translation or position the pose describes
        roation : pyquaternion.Quaternion
            rotation the pose describes as Quaternion
        frame_id : str (optinal defaul = '')
            name of the coordinate system the pose is defined
        normalize_roation : bool (optinal default = False)
            If true quaternion normalization is performed in the
            initialization process

        Returns
        ------
        Pose
            An instance of class Pose

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

        Examples
        --------
        Create a Pose by translation and quaternion and
        perform normalization
        >>> quaternion = Quaternion(matrix=np.eye(3))
        >>> translation = np.array([1.0,1.0,1.0])
        >>> p1 = Pose(translation, quaternion, _, True)
        """

        # translation
        try:
            self.__translation = np.fromiter(translation, float)
            if self.__translation.shape[0] != 3:
                raise ValueError('provided translation is not'
                                 ' convertabel to a numpy vector of size (3,)')
        except (TypeError, ValueError) as exc:
            raise exc

        # rotation
        if isinstance(rotation, Quaternion):
            self.__quaternion = Quaternion(rotation.q.copy())

        else:
            raise TypeError('rotation is not of the expected'
                            ' type Quaternion')

        if not isinstance(frame_id, str):
            raise TypeError('frame_id is not of expected type str')

        self.__frame_id = frame_id

        if not isinstance(normalize_rotation, bool):
            raise TypeError('normalize_rotation is not of expected type bool')

        self.__normalize_rotation = normalize_rotation

        if self.__normalize_rotation is True:
            self._normalize_rotation()

        self.__translation.flags.writeable = False
        self.__quaternion.q.flags.writeable = False

    @classmethod
    def from_transformation_matrix(cls, matrix, frame_id='',
                                   normalize_rotation=False):
        """
        Initialization of the pose class from transformation matrix

        Parameters
        ----------
        matrix : numpy.ndarray((4,4),np.dtype=floating)
            A transformation matrix in homogenous coordinates
        frame_id : str (optinal defaul = '')
            name of the coordinate system the pose is defined
        normalize_roation : bool (optinal default = False)
            If true quaternion normalization is performed in the
            initialization process

        Returns
        ------
        Pose
            An instance of class Pose

        Raises
        ------
        TypeError : type mismatch
            If tranformation matrix is not a 4x4 numpy array 
            with dtype floating

         Examples
        --------
        Create a Pose instance from transformation matrix and
        set frame_id
        >>> transformation_matrix = np.eye(4)
        >>> p0 = Pose.from_transformation_matrix(transformation_matrix, "global")
        """
        if isinstance(matrix, np.ndarray):
            if (len(matrix.shape) != 2 or
                    matrix.shape[0] != 4 or
                    matrix.shape[1] != 4):
                raise ValueError('matrix is not a 4x4 numpy array')
            if not issubclass(matrix.dtype.type, np.floating):
                raise TypeError('matrix is not a'
                                'dtype is no floating type')

            transformation_matrix = matrix.copy()
            translation = transformation_matrix[:-1, 3]
            try:
                quaternion = Quaternion(
                    matrix=transformation_matrix)
            except ValueError as exc:
                raise_from(ValueError(
                    'quaternion initialization went wrong'), exc)

        else:
            raise TypeError('matrix is not of type numpy array')

        return cls(translation, quaternion, frame_id, normalize_rotation)

    @classmethod
    def from_posestamped_msg(cls, msg):
        """
        Initialize Pose from ROS posestamped message

        Parameters
        ----------
        msg : PoseStamped from ROS geometry_msgs
            posestamped message 

        Returns
        -------
        Pose
            Instance of Pose generated from PoseStamped message

        Raises
        ------
        TypeError
            If msg is not of type PoseStamped
        """
        if not isinstance(msg, PoseStamped):
            raise TypeError('msg is not of expected type PoseStamped')

        translation = np.fromiter([msg.pose.position.x,
                                   msg.pose.position.y,
                                   msg.pose.position.z],
                                  float)

        quaternion = Quaternion([msg.pose.orientation.w,
                                 msg.pose.orientation.x,
                                 msg.pose.orientation.y,
                                 msg.pose.orientation.z])

        return cls(translation, quaternion)

    @property
    def frame_id(self):
        """
        frame_id : str (readonly)
            Id of the coordinate system / frame
        """
        return self.__frame_id

    @property
    def translation(self):
        """
        translation : numpy.array((3,) dtype=floating) (readonly)
            numpy row array of size 3 which describes the translation
        """
        return self.__translation

    @property
    def quaternion(self):
        """
        quaternion: Quaternion(pyquaternion lib) (readonly)
            Quaternion which describes the rotation
        """
        return self.__quaternion

    def normalize_roation(self):
        """
        Creates an instance of Pose with normalized quaternion

        Returns
        ------
            Instance of Pose with normalized quaternion / rotation
        """
        if self.__normalize_rotation:
            return self
        else:
            return Pose(self.__translation, self.__quaternion,
                        normalize_rotation=True)

    def _normalize_rotation(self):
        self.__quaternion._fast_normalise()
        if self.__quaternion[0] < 0:
            self.__quaternion = -self.__quaternion

    def is_roation_normalized(self):
        """
        Returns True is roation is normalized

        Returns
        -------
        result : bool
            True is roation is normalized else False
        """

        if self.__normalize_rotation:
            return True
        else:
            return False

    def rotation_matrix(self):
        """
        Returns the roation matrix in homogenous coordinates (4x4 numpy array)

        Returns
        ------
            A 4x4 numpy array with dtype float which represents the roation
            matrix in homogenous coordinates
        """

        return self.__quaternion.transformation_matrix
        return

    def transformation_matrix(self):
        """
        Return the transformation martix in homogenous coordinates (4x4 numpy array)

        Returns
        ------
            A 4x4 numpy array with dtype float which represents the transformation
            matrix in homogenous coordinates
        """
        transformation_matrix = self.__quaternion.transformation_matrix
        transformation_matrix[:-1, -1] = self.__translation
        return transformation_matrix

    def to_posestamped_msg(self):
        """
        Creates an instance of the ROS message PoseStamped

        Returns
        ------
            Instance of ROS message PoseStamped (seq and time are not set)
            docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseStamped.html

        """

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.__frame_id

        pose_stamped.position.x = self.__translation[0]
        pose_stamped.position.y = self.__translation[1]
        pose_stamped.position.z = self.__translation[2]

        pose_stamped.orientation.w = self.__quaternion[0]
        pose_stamped.orientation.x = self.__quaternion[1]
        pose_stamped.orientation.y = self.__quaternion[2]
        pose_stamped.orientation.z = self.__quaternion[3]

        return pose_stamped

    def __str__(self):
        axes = tuple(['w', 'x', 'y', 'z'])

        translation_str = '\n'.join(['translation.'+axes[i+1] + ' : '
                                     + str(value) for i, value
                                     in enumerate(self.__translation)])
        quaternion_str = '\n'.join(['quaternion.'+axes[i] + ' : ' + str(value)
                                    for i, value
                                    in enumerate(self.__quaternion)])

        return 'Pose:\n' + translation_str + '\n' + quaternion_str

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        r_tol = 1.0e-13
        a_tol = 1.0e-14

        if not isinstance(other, self.__class__):
            return False

        if id(other) == id(self):
            return True

        if not np.allclose(self.__translation, other.tranlation,
                           rtol=r_tol, atol=a_tol):
            return False

        if self.__quaternion != other.quaternion:
            return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)

    def __mul__(self, other):

        matrix_self = self.transformation_matrix()
        if isinstance(other, self.__class__):
            matrix_other = other.transformation_matrix()
            product = np.matmul(matrix_self, matrix_other)
            return self.from_transformation_matrix(product, self.__frame_id)
        elif (isinstance(other, np.ndarray) and
                issubclass(other.dtype.type, np.floating)):
            if other.shape in [(3,), (3, 1)]:
                vector = np.append(other, [[1.0]], axis=0)
                return np.matmul(matrix_self, vector)
            elif other.shape == (1, 3):
                vector = np.append(other, [[1.0]], axis=1)
                return np.matmul(matrix_self, vector.T)
            elif other.shape in [(4,), (4, 1)]:
                return np.matmul(matrix_self, other)
            elif other.shape == (1, 4):
                return np.matmul(matrix_self, other.T)
            elif other.shape == (4, 4):
                product = np.matmul(matrix_self, other)
                return self.from_transformation_matrix(product, self.__frame_id)
            else:
                TypeError('vector is not of shape (3,), (3,1)'
                          '(1,3), (4,), (4,1) or (1,4) or matrix (4,4)')

        else:
            TypeError('other is not of expected type Pose or'
                      ' 3 dimensional numpy vector or'
                      ' 4x4 transformation matrix dtype floating')

    def __rmul__(self, other):
        return np.dot(other, self.transformation_matrix())

    __array_priority__ = 10000
