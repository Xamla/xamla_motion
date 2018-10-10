# collision_object.py
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
from ..xamla_motion_exceptions import ArgumentError
from enum import Enum, unique
from typing import Iterable, TypeVar
import moveit_msgs.msg as moveit_msgs
import shape_msgs.msg as shape_msgs
import numpy as np

float_convertable = TypeVar('float_convertable', str, float, int)


@unique
class CollisionPrimitiveKind(Enum):
    plane = 0
    box = 1
    sphere = 2
    cylinder = 3
    cone = 4


class CollisionPrimitive(object):
    """
    Class CollisionPrimitive

    At the moment following simple primitives are supported:
    plane, box, sphere, cylinder, cone

    Methods
    -------
    from_shape_msg(msg, pose)
        Creates an Instance of CollisionPrimitive from moveit shape msg
    create_plane(a, b, c, d, pose)
        Creates an instance of CollisionPrimitive which represents a plane
    create_box(cls, x, y, z, pose)
        Creates an instance of CollisionPrimitive which represents a box
    create_unit_box(pose)
        Creates an instance of CollisionPrimitive which represents unit box
    create_sphere(radius, pose)
        Creates an instance of CollisionPrimitive which represents a sphere
    create_unit_sphere(pose)
        Creates an instance of CollisionPrimitive which represents unit sphere
    create_cylinder(height, radius, pose)
        Creates an instance of CollisionPrimitive which represents a cylinder
    create_cone(height, radius, pose)
        Creates an instance of CollisionPrimitive which represents a cone
    to_shape_msg()
        Creates an instance of moveit/shape_msgs.Plane for a plane primitive
        or an instance of moveit/shape_msgs.SolidPrimitive for all other primitive
    """

    expected_parameter_count = {CollisionPrimitiveKind.plane: 4,
                                CollisionPrimitiveKind.box: 3,
                                CollisionPrimitiveKind.sphere: 1,
                                CollisionPrimitiveKind.cylinder: 2,
                                CollisionPrimitiveKind.cone: 2}

    def __init__(self, kind: CollisionPrimitiveKind, parameters: Iterable[float_convertable], pose: Pose):
        """
        Initialization of CollisionPrimitive

        Representation of a plane, using the plane equation ax + by + cz + d = 0
            a := parameters[0]
            b := parameters[1]
            c := parameters[2]
            d := parameters[3]

        Representation of a box
            x := parameters[0]
            y := parameters[1]
            z := parameters[2]

        Representation of a sphere
        radius := parameters[0]

        Representation of a cylinder
            height := parameters[0]
            radius := parameters[1]

        Representation of a cone
            height := parameters[0]
            radius := parameters[1]

        Parameters
        ----------
        kind : CollisionPrimitiveKind
            Specifices which kind of primitive the new instance should represent
        parameters : Iterable[float convertable]
            Parameters which describe the primitive
        pose : Pose
            pose describes the position and orientation of the mid point of the primitive

        Returns
        -------
        CollisionPrimitive
            An Instance of collision primitive

        Raises
        ------
        TypeError
            If kind is not of type CollisionPrimitiveKind
            If pose is not of type Pose
            If parameters is not of type iterable of float convertable
        ValuesError
            If for a specific primitive kind the wrong number of parameters
            are provided
            If kind is CollisionPrimitiveKind.plane and the normal vector
            is described only by zeros
            If kind is not CollisionPrimitveKind.plane and a parameter in
            parameters is smaller than zero

        Examples
        --------

        Create a instance of a box collision primitive which is one meter in size
        in all three dimensions and the origin is the origin of the world 
        coordinate system

        >>> import numpy as np
        >>> from xamla_motion.data_types import CollisionPrimitive, CollisionPrimitiveKind, Pose
        >>> origin = Pose.from_transformation_matrix(np.eye(4),'world')
        >>> CollisionPrimitive(CollisionPrimitiveKind.box,[1.0,1.0,1.0], origin)
        CollisionPrimitive
        pose = Pose:
        translation.x : 0.0
        translation.y : 0.0
        translation.z : 0.0
        quaternion.w : 1.0
        quaternion.x : 0.0
        quaternion.y : 0.0
        quaternion.z : 0.0
        frame_id : world
        kind = CollisionPrimitiveKind.box
        parameters = [ 1.  1.  1.]

        """

        if not isinstance(kind, CollisionPrimitiveKind):
            raise TypeError('kind is nof of expected type'
                            ' CollisionPrimitiveKind')

        if not isinstance(pose, Pose):
            raise TypeError('pose is not of expected type Pose')

        if self.expected_parameter_count[kind] != len(parameters):
            raise ValueError('expected ' + self.expected_parameter_count[kind]
                             + ' parameters for primitive ' + str(kind) +
                             ' but provide ' + len(parameters))

        self.__parameters = np.fromiter(parameters, float)

        if kind is CollisionPrimitiveKind.plane:
            if np.allclose(self.__parameters[:3], 0.0):
                raise ValueError('invalid normal vector for plane primitive, parameters:' +
                                 ', '.join('{:0.2f}'.format(i) for i in self.__parameters))
        else:
            if any(self.__parameters < 0.0):
                raise ValueError('parameters for collision primitive' +
                                 str(kind) + 'must not be negative, parameters: ' +
                                 ', '.join('{:0.2f}'.format(i) for i in self.__parameters))
        self.__kind = kind
        self.__pose = pose

    @classmethod
    def from_shape_msg(cls, msg, pose):
        """
        Creates an Instance of CollisionPrimitive from moveit shape msg

        At the moment it can handle moveit plane and solid primitive

        Parameters
        ----------
        msg : message type from moveit/shape_msgs
            Message from which a CollisionPrimitive should be created
        pose : Pose
            Pose instance which describes the midpoint pose of primitive
        """
        if isinstance(msg, shape_msgs.Plane):
            cls(CollisionPrimitiveKind.plane, msg.coef, pose)
        elif isinstance(msg, shape_msgs.SolidPrimitive):
            cls(CollisionPrimitiveKind(msg.type), msg.dimensions, pose)

    @classmethod
    def create_plane(cls, a: float, b: float, c: float, d: float, pose: Pose):
        """
        Creates an instance of CollisionPrimitive which represents a plane

        Representation of a plane, using the plane equation ax + by + cz + d = 0

        Parameters
        ----------
        a: float convertable
            a parameter of plane equation
        b: float convertable
            b parameter of plane equation
        c: float convertable
            c parameter of plane equation
        d: float convertable
            d parameter of plane equation
        pose: Pose
            Pose of the midpoint of the plane

        Returns
        -------
        CollisionPrimitve
            Primitive which represents a plane

        Raises
        ------
        TypeError
            If pose is not of type Pose
            If a parameter is not float convertable
        ValuesError
            If the normal vector is described only by zeros
            If kind is not CollisionPrimitveKind.plane and a parameter in
            parameters is smaller than zero
        """
        return cls(CollisionPrimitiveKind.plane, [a, b, c, d], pose)

    @classmethod
    def create_box(cls, x: float, y: float, z: float, pose: Pose):
        """
        Creates an instance of CollisionPrimitive which represents a box

        Representation of box aligned with pose xyz coordinates

        Parameters
        ----------
        x: float convertable
            x coordinate of pose
        y: float convertable
            y coordinate of pose
        z: float convertable
            z coordinate of pose
        pose: Pose
            Pose of the midpoint of the box

        Returns
        -------
        CollisionPrimitve
            Primitive which represents a box

        Raises
        ------
        TypeError
            If pose is not of type Pose
            If a parameter is not float convertable
        ValuesError
            If  a parameter in is smaller than zero
        """
        return cls(CollisionPrimitiveKind.box, [x, y, z], pose)

    @classmethod
    def create_unit_box(cls, pose: Pose):
        """
        Creates an instance of CollisionPrimitive which represents unit box

        Parameters
        ----------
        pose: Pose
            Pose of the midpoint of the box

        Returns
        -------
        CollisionPrimitve
            Primitive which represents a unit box

        Raises
        ------
        TypeError
            If pose is not of type Pose
            If a parameter is not float convertable
        ValuesError
            If  a parameter in is smaller than zero
        """
        return cls(CollisionPrimitiveKind.box, [1.0, 1.0, 1.0], pose)

    @classmethod
    def create_sphere(cls, radius: float, pose: Pose):
        """
        Creates an instance of CollisionPrimitive which represents a sphere

        Parameters
        ----------
        radius: float convertable
            radius of the sphere
        pose: Pose
            Pose of the midpoint of the sphere

        Returns
        -------
        CollisionPrimitve
            Primitive which represents a sphere

        Raises
        ------
        TypeError
            If pose is not of type Pose
            If a parameter is not float convertable
        ValuesError
            If  a parameter in is smaller than zero
        """
        return cls(CollisionPrimitiveKind.sphere, [radius], pose)

    @classmethod
    def create_unit_sphere(cls, pose: Pose):
        """
        Creates an instance of CollisionPrimitive which represents unit sphere

        Parameters
        ----------
        pose: Pose
            Pose of the midpoint of the sphere

        Returns
        -------
        CollisionPrimitve
            Primitive which represents a unit sphere

        Raises
        ------
        TypeError
            If pose is not of type Pose
            If a parameter is not float convertable
        ValuesError
            If  a parameter in is smaller than zero
        """
        return cls(CollisionPrimitiveKind.sphere, [1.0], pose)

    @classmethod
    def create_cylinder(cls, height: float, radius: float, pose: Pose):
        """
        Creates an instance of CollisionPrimitive which represents a cylinder

        Parameters
        ----------
        height : float convertable
            height of the cylinder
        radius : float convertable
            radius of the cylinder
        pose: Pose
            Pose of the midpoint of the cylinder

        Returns
        -------
        CollisionPrimitve
            Primitive which represents a cylinder

        Raises
        ------
        TypeError
            If pose is not of type Pose
            If a parameter is not float convertable
        ValuesError
            If  a parameter in is smaller than zero
        """
        return cls(CollisionPrimitiveKind.cylinder, [height, radius], pose)

    @classmethod
    def create_cone(cls, height: float, radius: float, pose: Pose):
        """
        Creates an instance of CollisionPrimitive which represents a cone

        Parameters
        ----------
        height : float convertable
            height of the cone
        radius : float convertable
            radius of the cone
        pose: Pose
            Pose of the midpoint of the cone

        Returns
        -------
        CollisionPrimitve
            Primitive which represents a cone

        Raises
        ------
        TypeError
            If pose is not of type Pose
            If a parameter is not float convertable
        ValuesError
            If  a parameter in is smaller than zero
        """
        return cls(CollisionPrimitiveKind.cone, [height, radius], pose)

    @property
    def kind(self):
        """
        kind: CollisionPrimitiveKind
            kind of the collision primitive
        """
        return self.__kind

    @property
    def parameters(self):
        """
        parameters: numpy.array(dtype.floating)
            parameters which describe the primitive
        """
        return self.__parameters

    @property
    def pose(self):
        """
        pose: Pose
            Describes the position and orientation of the primitive for the primitive midpoint
        """
        return self.__pose

    def to_shape_msg(self):
        """
        Creates an instance of moveit/shape_msgs.Plane for a plane primitive
        or an instance of moveit/shape_msgs.SolidPrimitive for all other primitive
        """

        if self.__kind is CollisionPrimitiveKind.plane:
            msg = shape_msgs.Plane()
            msg.coef = list(self.__parameters)
        else:
            msg = shape_msgs.SolidPrimitive()
            msg.type = self.__kind.value
            msg.dimensions = list(self.__parameters)

        return msg

    def __str__(self):
        s = '\n'.join(['  '+k+' = ' + str(v)
                       for k, v in self.__dict__.items()])
        s = s.replace('_'+self.__class__.__name__+'__', '')
        return self.__class__.__name__+'\n'+s

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False

        if id(other) == id(self):
            return True

        if other.kind != self.__kind:
            return False

        if not np.allclose(other.parameters, self.__parameters):
            return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)


class CollisionObject(object):
    """
    Class CollisionObject

    An Instance of this class represent a collision object.
    A collision object can be a simple primitive or a assembly of
    many primitives

    Methods
    -------
    from_collision_object_msg(msg)
        Create a Pose from an instance of moveit_msgs/CollisionObject
    to_collision_object_msg(self, action):
        Creates a ROS collision object message from this instance
    """

    def __init__(self, primitives: Iterable[CollisionPrimitive], frame_id: str='world'):
        """
        Initialization of CollisionObject class

        Parameters
        ----------
        primitives : Iterable[CollisionPrimitives] or None
            primitives which describe the collision object
        frame_id : str
            Frame / coordinate system in which the
            collision object is described

        Returns
        -------
        CollisionObject
            Instance of CollisionObject

        Raises
        ------
        TypeError
            If primitives are not of type iterable of
            CollisionPrimitives or None or
            if frame_id is not str convertable

        """

        if primitives and any(not isinstance(p, CollisionPrimitive)
                              for p in primitives):
            raise TypeError('primitives is not of type iterable of'
                            ' CollisionPrimitives or empty')

        self.__frame_id = str(frame_id)
        self.__primitives = list(primitives)

    @classmethod
    def from_collision_object_msg(cls, msg):
        """
        Create a Pose from an instance of moveit_msgs/CollisionObject

        Parameters
        ----------
        msg : ros message moveit_msgs/CollisionObject
            Message of moveit_msgs/CollisionObject


        Returns
        -------
        CollisionObject
            New instance of collision object from msg

        Raises
        ------
        TypeError
            If msg is not of type moveit_msgs/CollisionObject
        """

        primitives = []

        if not isinstance(msg, moveit_msgs.CollisionObject):
            raise TypeError('msg is not of type  moveit_msgs/CollisionObject')

        for i, p in enumerate(msg.planes):
            primitives.append(CollisionPrimitive(CollisionPrimitiveKind.plane,
                                                 p.coef,
                                                 Pose.from_pose_msg(msg.plane_poses[i],
                                                                    msg.header.frame_id)))

        for i, p in enumerate(msg.primitives):
            primitives.append(CollisionPrimitive(CollisionPrimitiveKind(p.type),
                                                 p.dimensions,
                                                 Pose.from_pose_msg(msg.primitive_poses[i],
                                                                    msg.header.frame_id)))

        return cls(primitives, msg.header.frame_id)

    @property
    def frame_id(self):
        """
        frame_id : str (read only)
            Frame / coordinate system in which the
            collision object is described
        """
        return self.__frame_id

    @property
    def primitives(self):
        """
        primitives : List[CollisionPrimitives] or None (read only)
            primitives which describe the collision object
        """
        return self.__primitives

    def to_collision_object_msg(self, action):
        """
        Creates a ROS collision object message from this instance

        Parameters
        ----------
        action : moveit_msg/CollisionObject operation
            Defines which action/opertation should be performed
            (add, remove, append, move)

        Returns
        -------
        moveit_msgs/CollisionObject 
        """

        msg = moveit_msgs.CollisionObject()
        msg.header.frame_id = self.__frame_id

        planes = []
        plane_poses = []
        primitives = []
        primitive_poses = []

        for p in self.__primitives:
            if p.kind is CollisionPrimitiveKind.plane:
                plane_poses.append(p.pose.to_pose_msg())
                planes.append(p.to_shape_msg())
            else:
                primitive_poses.append(p.pose.to_pose_msg())
                primitives.append(p.to_shape_msg())

        msg.planes = planes
        msg.plane_poses = plane_poses
        msg.primitives = primitives
        msg.primitive_poses = primitive_poses
        msg.meshes = []
        msg.mesh_poses = []
        msg.operation = action

        return msg

    def __len__(self):
        return len(self.__primitives)

    def __iter__(self):
        return iter(self.__primitives)

    def __str__(self):
        return str(self.__primitives)

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False

        if id(other) == id(self):
            return True

        if other.frame_id != self.__frame_id:
            return False

        for i, p in enumerate(other):
            if p != self.__primitives[i]:
                return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)
