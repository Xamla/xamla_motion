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

from copy import deepcopy
from pyquaternion import Quaternion
import geometry_msgs.msg as geometry_msgs
import numpy as np


class Pose(object):
    """
    Pose defined by three dimensional translation and rotation

    The translation is representated by a three dimensional
    row vector. The rotation is represented by a
    quaternion. The pose itself is defined in coordinate system
    which is defined in frame_id (default world)

    Methods
    -------
    from_transformation_matrix(matrix, frame_id='',normalize_rotation=False)
        Creates an instance of Pose from a transformation matrix
    from_posestamped_msg(msg)
        Initialize Pose from ROS posestamped message
    from_pose_msg(msg, frame_id='')
        Initialize Pose from ROS pose message
    normalize_rotation()
        Creates an instance of Pose with normalized quaternion
    is_rotation_normalized()
        Return True if quaternion is normalized
    rotation_matrix()
        Returns the rotation matrix(3x3 numpy array)
    transformation_matrix()
        Returns the transformation matrix in homogenous coordinates (4x4 numpy array)
    inverse(new_frame_id)
        Creates an instance which contains the inverse of this pose
    translate(translation)
        Translate pose
    rotate(rotation)
        Rotate pose
    to_posestamped_msg()
        Creates an instance of the ROS message PoseStamped from Pose
    """

    def __init__(self, translation, rotation, frame_id='world',
                 normalize_rotation=False):
        """
        Initialization of the pose class

        For the internal quaternion representation and also to initialize the
        the class the library pyquaternion is used :
        https://kieranwynn.github.io/pyquaternion/

        Parameters
        ----------
        translation : convertable to numpy array of shape (3,)
            translation or position the pose describes
        rotation : pyquaternion.Quaternion
            rotation the pose describes as Quaternion
        frame_id : str (optinal defaul = '')
            name of the coordinate system the pose is defined
        normalize_rotation : bool (optinal default = False)
            If true quaternion normalization is performed in the
            initialization process

        Returns
        ------
        pose : Pose
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

        if normalize_rotation is True:
            self._normalize_rotation()

        self.__translation.flags.writeable = False
        self.__quaternion.q.flags.writeable = False

    @classmethod
    def from_transformation_matrix(cls, matrix, frame_id='world',
                                   normalize_rotation=False):
        """
        Initialization of the pose class from transformation matrix

        Parameters
        ----------
        matrix : numpy.ndarray((4,4),np.dtype=floating)
            A transformation matrix in homogenous coordinates
        frame_id : str (optional defaul = '')
            name of the coordinate system the pose is defined
        normalize_rotation : bool (optional default = False)
            If true quaternion normalization is performed in the
            initialization process

        Returns
        ------
        pose : Pose
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
                raise ValueError(
                    'quaternion initialization went wrong') from exc

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
        pose : Pose
            Instance of Pose generated from PoseStamped message

        Raises
        ------
        TypeError
            If msg is not of type PoseStamped
        """
        if not isinstance(msg, geometry_msgs.PoseStamped):
            raise TypeError('msg is not of expected type PoseStamped')

        translation = np.fromiter([msg.pose.position.x,
                                   msg.pose.position.y,
                                   msg.pose.position.z],
                                  float)

        quaternion = Quaternion([msg.pose.orientation.w,
                                 msg.pose.orientation.x,
                                 msg.pose.orientation.y,
                                 msg.pose.orientation.z])

        frame_id = msg.header.frame_id

        if not frame_id:
            frame_id = 'world'

        return cls(translation, quaternion, frame_id)

    @classmethod
    def from_pose_msg(cls, msg, frame_id='world'):
        """
        Initialize Pose from ros geometry_msgs/Pose

        Parameters
        ----------
        msg : Pose from geometry_msgs
            pose message

        Returns
        -------
        pose : Pose
            Instance of Pose from pose message

        Raises
        ------
        TypeError
            If msg is not of type Pose
        """

        if not isinstance(msg, geometry_msgs.Pose):
            raise TypeError('msg is not of type ros geometry_msgs/Pose')

        translation = np.fromiter([msg.position.x,
                                   msg.position.y,
                                   msg.position.z],
                                  float)

        quaternion = Quaternion([msg.orientation.w,
                                 msg.orientation.x,
                                 msg.orientation.y,
                                 msg.orientation.z])

        return cls(translation, quaternion, frame_id)

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

    def normalize_rotation(self):
        """
        Creates an instance of Pose with normalized quaternion

        Returns
        ------
        pose : Pose
            Instance of Pose with normalized quaternion / rotation
        """
        if np.isclose(np.linalg.norm(self.__quaternion.elements), 1.0):
            return self
        else:
            return Pose(self.__translation, self.__quaternion,
                        normalize_rotation=True)

    def _normalize_rotation(self):
        self.__quaternion._fast_normalise()
        if self.__quaternion[0] < 0:
            self.__quaternion = -self.__quaternion

    def is_rotation_normalized(self):
        """
        Returns True is rotation is normalized

        Returns
        -------
        result : bool
            True is rotation is normalized else False
        """

        if np.isclose(np.linalg.norm(self.__quaternion.elements), 1.0):
            return True
        else:
            return False

    def rotation_matrix(self):
        """
        Returns the rotation matrix (3x3 numpy array)

        Returns
        ------
        rotation_matrix : np.ndarray
            A 3x3 numpy array with dtype float which represents the rotation
            matrix
        """

        return self.__quaternion.rotation_matrix

    def transformation_matrix(self):
        """
        Return the transformation martix in homogenous coordinates (4x4 numpy array)

        Returns
        ------
        transformation_matrix : np.ndarray
            A 4x4 numpy array with dtype float which represents the transformation
            matrix in homogenous coordinates
        """
        transformation_matrix = self.__quaternion.transformation_matrix
        transformation_matrix[:-1, -1] = self.__translation
        return transformation_matrix

    def inverse(self, new_frame_id):
        """
        Creates an instance which contains the inverse of this pose

        Parameters
        ----------
        new_frame_id : str convertable
            name of the coordinate system in which pose is now defined 

        Returns
        -------
        inv_pose : Pose
            pose which is the inverse of self

        Raises
        ------
        TypeError : type mismatch
            If new_frame_id is not of type str
        """
        q_inv = self.__quaternion.inverse
        t_inv = q_inv.rotate(-self.translation)
        return self.__class__(t_inv, q_inv, new_frame_id)

    def translate(self, translation):
        """
        Translate pose

        Parameters
        ----------
        translation : Iterable
            translation which defines the change in x,y,z

        Returns
        -------
        translated_pose : Pose
            pose which is translated by translate vector

        Raises
        ------
        TypeError : type mismatch
            If translation is not convertable to np.ndarray
        ValueError
            If translation is not of size 3 (x,y,z)
        """

        try:
            translation = np.fromiter(translation, float)
            if translation.shape[0] != 3:
                raise ValueError('provided translation is not'
                                 ' convertabel to a numpy vector of size (3,)')
        except (TypeError, ValueError) as exc:
            raise exc

        new_pose = deepcopy(self)
        new_pose._Pose__translation += translation
        return new_pose

    def rotate(self, rotation):
        """
        Rotate pose

        Parameters
        ----------
        rotation: (Quaternion or np.ndarray)
            defines rotation by Quaternion or 3x3 rotation matrix

        Returns
        -------
        rotated_pose : Pose
            pose which is rotated by rotation

        Raises
        ------
        TypeError : type mismatch
            If rotation is not one of expected types
            3x3 np.ndarray or pyquaternion.Quaternion
        ValueError
            If initialization of Quaternion is not possible
        """
        is_quaternion = isinstance(rotation, Quaternion)
        is_rotation_matrix = (isinstance(rotation, np.ndarray) and
                              rotation.shape == (3, 3))

        if is_quaternion:
            new_q = self.__quaternion * rotation
        elif is_rotation_matrix:
            try:
                new_q = self.__quaternion * Quaternion(matrix=rotation)
            except ValueError as exc:
                raise ValueError(
                    'quaternion initialization went wrong') from exc
        else:
            raise TypeError('rotation is not one of expected types '
                            'Quaternion or np.ndarray (3x3)')

        new_pose = deepcopy(self)
        new_pose._Pose__quaternion = new_q
        return new_pose

    def to_posestamped_msg(self):
        """
        Creates an instance of the ROS message PoseStamped

        Returns
        ------
            Instance of ROS message PoseStamped (seq and time are not set)
            docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseStamped.html

        """

        pose_stamped = geometry_msgs.PoseStamped()
        pose_stamped.header.frame_id = self.__frame_id

        pose_stamped.pose.position.x = self.__translation[0]
        pose_stamped.pose.position.y = self.__translation[1]
        pose_stamped.pose.position.z = self.__translation[2]

        pose_stamped.pose.orientation.w = self.__quaternion[0]
        pose_stamped.pose.orientation.x = self.__quaternion[1]
        pose_stamped.pose.orientation.y = self.__quaternion[2]
        pose_stamped.pose.orientation.z = self.__quaternion[3]

        return pose_stamped

    def to_pose_msg(self):
        """
        Creates an instance of the ROS message Pose

        Returns
        ------
            Instance of ROS message Pose
            geometry_msgs/Pose

        """

        pose = geometry_msgs.Pose()

        pose.position.x = self.__translation[0]
        pose.position.y = self.__translation[1]
        pose.position.z = self.__translation[2]

        pose.orientation.w = self.__quaternion[0]
        pose.orientation.x = self.__quaternion[1]
        pose.orientation.y = self.__quaternion[2]
        pose.orientation.z = self.__quaternion[3]

        return pose

    def __str__(self):
        axes = tuple(['w', 'x', 'y', 'z'])

        translation_str = '\n'.join(['translation.'+axes[i+1] + ' : '
                                     + str(value) for i, value
                                     in enumerate(self.__translation)])
        quaternion_str = '\n'.join(['quaternion.'+axes[i] + ' : ' + str(value)
                                    for i, value
                                    in enumerate(self.__quaternion)])

        return ('Pose:\n' + translation_str + '\n' + quaternion_str +
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

        if not np.allclose(self.__translation, other.tranlation,
                           rtol=r_tol, atol=a_tol):
            return False

        if self.__quaternion != other.quaternion:
            return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)

    def __mul__(self, other):

        if isinstance(other, self.__class__):
            new_q = self.__quaternion * other.quaternion
            new_t = (self.__translation +
                     self.__quaternion.rotate(other.translation))
            return self.__class__(new_t, new_q, self.frame_id)
        elif (isinstance(other, np.ndarray) and
                issubclass(other.dtype.type, np.floating)):
            if other.shape == (3,):
                new_t = (self.__translation +
                         self.__quaternion.rotate(other))
                return new_t
            elif other.shape == (3, 1):
                new_t = (self.__translation +
                         self.__quaternion.rotate(other))
                return np.expand_dims(new_t, axis=1)
            elif other.shape == (4,):
                new_t = np.ones(other.shape)
                new_t[0:3] = (self.__translation * other[-1] +
                              self.__quaternion.rotate(other[:3]))
                return new_t
            elif other.shape == (4, 1):
                new_t = np.ones(other.shape)
                t3 = (self.__translation * other[-1] +
                      self.__quaternion.rotate(other[:3]))
                new_t[0:3] = np.expand_dims(t3, axis=1)
                return new_t
            elif other.shape == (4, 4):
                product = np.matmul(self.transformation_matrix(), other)
                return self.from_transformation_matrix(product, self.__frame_id)
            else:
                TypeError('vector is not of shape (3,), (3,1),'
                          ' (4,), (4,1)  or matrix (4,4)')

        else:
            TypeError('other is not of expected type Pose,'
                      ' 3 or 4 dimensional numpy vector or'
                      ' 4x4 transformation matrix dtype floating')

    def __rmul__(self, other):
        return np.dot(other, self.transformation_matrix())

    __array_priority__ = 10000
