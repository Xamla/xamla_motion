import pytest
from xamla_motion.data_types import (Pose,
                                     JointSet,
                                     JointValues,
                                     CartesianPath,
                                     CollisionObject,
                                     CollisionPrimitive
                                     )
from xamla_motion.v2 import WorldViewClient
from xamla_motion.xamla_motion_exceptions import ArgumentError
import numpy as np


class TestWorldViewClient(object):

    @classmethod
    def setup_class(cls):
        cls.client = WorldViewClient()

        joint_set = JointSet('1,2,3')

        cls.joint_values_1 = JointValues(joint_set, 0.0)
        cls.joint_values_2 = JointValues(joint_set, [1, 2, 3])

        cls.pose_1 = Pose.identity()
        cls.pose_2 = Pose.identity().translate([0.1, 0.2, 1.8])
        cls.pose_3 = Pose.identity().translate([0.1, -0.2, 1.8])

        cls.cartesian_path_1 = CartesianPath([cls.pose_1, cls.pose_2])
        cls.cartesian_path_2 = CartesianPath([cls.pose_2, cls.pose_1])

        collision_box_1 = CollisionPrimitive.create_box(
            0.05,
            0.05,
            0.05,
            cls.pose_1
        )

        collision_box_2 = CollisionPrimitive.create_box(
            0.05,
            0.05,
            0.05,
            cls.pose_2
        )

        collision_box_3 = CollisionPrimitive.create_box(
            0.05,
            0.05,
            0.05,
            cls.pose_3
        )

        cls.collision_object_1 = CollisionObject([collision_box_1])
        cls.collision_object_2 = CollisionObject(
            [collision_box_2, collision_box_3])

        cls.folder_path = 'test/my_test'
        cls.joint_values_path = 'test_joint_values'
        cls.pose_path = 'test_pose'
        cls.cartesian_path = 'test_cartesian_path'
        cls.collision_object_path = 'test_collision_object'

    def test_add_folder(self):
        self.client.add_folder(self.folder_path)

        with pytest.raises(ArgumentError):
            self.client.add_folder(
                self.folder_path,
                raise_exception_if_exists=True
            )

    def test_add_joint_values(self):
        with pytest.raises(TypeError):
            self.client.add_joint_values(1, self.joint_values_1)

        with pytest.raises(TypeError):
            self.client.add_joint_values(self.joint_values_path, self.pose_1)

        self.client.add_joint_values(
            self.joint_values_path,
            self.joint_values_1
        )

        with pytest.raises(ArgumentError):
            self.client.add_joint_values(
                self.joint_values_path,
                self.joint_values_1,
                update_if_exists=False
            )

        self.client.add_joint_values(
            self.joint_values_path,
            self.joint_values_2,
        )

    def test_get_joint_values(self):
        joint_values = self.client.get_joint_values(self.joint_values_path)

        assert joint_values == self.joint_values_2

    def test_add_pose(self):
        with pytest.raises(TypeError):
            self.client.add_pose(1, self.pose_1)

        with pytest.raises(TypeError):
            self.client.add_pose(self.pose_path, self.joint_values_1)

        self.client.add_pose(
            self.pose_path,
            self.pose_1
        )

        with pytest.raises(ArgumentError):
            self.client.add_pose(
                self.pose_path,
                self.pose_1,
                update_if_exists=False
            )

        self.client.add_pose(
            self.pose_path,
            self.pose_2,
        )

    def test_get_pose(self):
        pose = self.client.get_pose(self.pose_path)

        assert pose == self.pose_2

    def test_add_cartesian_path(self):
        with pytest.raises(TypeError):
            self.client.add_cartesian_path(1, self.cartesian_path_1)

        with pytest.raises(TypeError):
            self.client.add_cartesian_path(
                self.cartesian_path, self.pose_1)

        self.client.add_cartesian_path(
            self.cartesian_path,
            self.cartesian_path_1
        )

        with pytest.raises(ArgumentError):
            self.client.add_cartesian_path(
                self.cartesian_path,
                self.cartesian_path_1,
                update_if_exists=False
            )

        self.client.add_cartesian_path(
            self.cartesian_path,
            self.cartesian_path_2,
        )

    def test_get_cartesian_path(self):
        cartesian_path = self.client.get_cartesian_path(self.cartesian_path)

        assert cartesian_path == self.cartesian_path_2

    def test_add_collision_object(self):
        with pytest.raises(TypeError):
            self.client.add_collision_object(1, self.collision_object_1)

        with pytest.raises(TypeError):
            self.client.add_collision_object(
                self.collision_object_path, self.pose_1)

        self.client.add_collision_object(
            self.collision_object_path,
            self.collision_object_1
        )

        with pytest.raises(ArgumentError):
            self.client.add_collision_object(
                self.collision_object_path,
                self.collision_object_1,
                update_if_exists=False
            )

        self.client.add_collision_object(
            self.collision_object_path,
            self.collision_object_2,
        )

    def test_get_collision_object(self):
        collision_object = self.client.get_collision_object(
            self.collision_object_path)

        assert collision_object == self.collision_object_2

    def test_remove_element(self):
        self.client.remove_element(self.folder_path)

        with pytest.raises(ArgumentError):
            self.client.remove_element(
                self.folder_path,
                raise_exception_if_not_exists=True
            )

        self.client.remove_element(self.joint_values_path)
        self.client.remove_element(self.pose_path)
        self.client.remove_element(self.cartesian_path)
        self.client.remove_element(self.collision_object_path)
