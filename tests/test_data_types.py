import pytest
from xamla_motion.data_types import JointSet, JointValues, JointTrajectoryPoint
from datetime import timedelta
import numpy as np


class TestJointTrajectoryPoint(object):

    @classmethod
    def setup_class(cls):
        joint_set1 = JointSet('joint1,joint2')
        joint_set2 = JointSet('joint3,joint4')
        joint_set3 = JointSet('joint3,joint1')
        joint_set4 = JointSet('joint5,joint6')

        joint_values1 = JointValues(joint_set1, 1.0)
        joint_values2 = JointValues(joint_set2, 2.0)
        joint_values3 = JointValues(joint_set3, 3.0)
        joint_values4 = JointValues(joint_set4, 4.0)

        timedelta1 = timedelta(hours=1, seconds=1)
        timedelta2 = timedelta(hours=1, seconds=2)

        cls.joint_trajectory_point1 = JointTrajectoryPoint(timedelta1,
                                                           joint_values1,
                                                           joint_values1)
        cls.joint_trajectory_point2 = JointTrajectoryPoint(timedelta1,
                                                           joint_values2,
                                                           joint_values2)
        cls.joint_trajectory_point3 = JointTrajectoryPoint(timedelta1,
                                                           joint_values3,
                                                           joint_values3)
        cls.joint_trajectory_point4 = JointTrajectoryPoint(timedelta1,
                                                           joint_values4,
                                                           joint_values4)
        cls.joint_trajectory_point5 = JointTrajectoryPoint(timedelta2,
                                                           joint_values2,
                                                           joint_values2)
        cls.joint_trajectory_point6 = JointTrajectoryPoint(timedelta1,
                                                           joint_values2,
                                                           joint_values2,
                                                           joint_values2)

    def test_valid_merge(self):
        merged = self.joint_trajectory_point1.merge(
            self.joint_trajectory_point2)
        assert merged.joint_set.names == ['joint1',
                                          'joint2',
                                          'joint3',
                                          'joint4']
        assert merged.positions == approx(np.array([1.0,
                                                    1.0,
                                                    2.0,
                                                    2.0]))

    def test_valid_merge_multiple_instances(self):
        merged = self.joint_trajectory_point1.merge([
            self.joint_trajectory_point2,)
            self.joint_trajectory_point4])
        assert merged.joint_set.names == ['joint1',
                                          'joint2',
                                          'joint3',
                                          'joint4',
                                          'joint5',
                                          'joint6']

        assert merged.positions == approx(np.array([1.0,
                                                    1.0,
                                                    2.0,
                                                    2.0,
                                                    4.0,
                                                    4.0]))

    def test_non_matching_time(self):
        with pytest.raise(ValueError):
            self.joint_trajectory_point1.merge(
            self.joint_trajectory_point5)

    def test_non_matching_time_multiple_instances(self):
        with pytest.raise(ValueError):
            merged = self.joint_trajectory_point1.merge([
                self.joint_trajectory_point4,)
                self.joint_trajectory_point5])

    def test_non_mergeable_joint_sets(self):
        with pytest.raise(ValueError):
            self.joint_trajectory_point1.merge(
            self.joint_trajectory_point3)

    def test_non_mergeable_joint_sets_multiple_instances(self):
        with pytest.raise(ValueError):
            merged = self.joint_trajectory_point1.merge([
                self.joint_trajectory_point2,)
                self.joint_trajectory_point3])

    def test_non_mergeable_properties(self):
        with pytest.raise(ValueError):
            self.joint_trajectory_point1.merge(
            self.joint_trajectory_point6)

    def test_non_mergeable_properties_multiple_instances(self):
        with pytest.raise(ValueError):
            merged = self.joint_trajectory_point1.merge([
                self.joint_trajectory_point4,)
                self.joint_trajectory_point6])
