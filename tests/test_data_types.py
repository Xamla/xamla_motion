import pytest
from xamla_motion.data_types import JointSet, JointValues, JointTrajectoryPoint, Pose
from datetime import timedelta
import numpy as np
from pyquaternion import Quaternion


class TestPose(object):

    @classmethod
    def setup_class(cls):
        c30 = np.sqrt(3)/2.0
        s30 = -0.5

        c60 = -s30
        s60 = -c30

        # create pose1 from transformation matrix
        m1 = np.eye(4)
        m1[0:3, 3] = np.asarray([0.8660254037844386, 0.5, 0.0])
        m1[0:2, 0:2] = np.asarray([[c30, -s30], [s30, c30]])
        cls.pose1 = Pose.from_transformation_matrix(m1)

        # create pose2 from transformation matrix
        m2 = np.eye(4)
        m2[0:3, 3] = np.asarray([0.25, -0.43301270189221935, 0.0])
        m2[0:2, 0:2] = np.asarray([[c60, -s60], [s60, c60]])
        cls.pose2 = Pose.from_transformation_matrix(m2)

        # create pose3 from transformation matrix
        cls.m3 = np.eye(4)
        cls.m3[0:3, 3] = np.asarray([0.8660254037844386, 0.0, 0.0])
        cls.m3[0:2, 0:2] = np.asarray([[0.0, 1.0], [-1.0, 0.0]])
        cls.pose3 = Pose.from_transformation_matrix(cls.m3)

    def test_pose_inverse(self):
        p_inv = self.pose1.inverse('new_frame')
        p_m = (self.pose1*p_inv).transformation_matrix()

        assert p_m == pytest.approx(np.eye(4))

    def test_translate(self):
        t_pose = self.pose3.translate([0.0, 1.0, 1.0])
        gt = self.m3.copy()
        gt[1, 3] = 1.0
        gt[2, 3] = 1.0
        assert t_pose.transformation_matrix() == pytest.approx(gt)

    def test_rotate(self):
        m = self.m3.copy()
        m[0:3, 0:3] = np.eye(3)
        p = Pose.from_transformation_matrix(m)
        r_pose = p.rotate(self.m3[0:3, 0:3])
        assert r_pose.transformation_matrix() == pytest.approx(self.m3)

    def test_pose_mul_pose(self):
        tri_pose = (self.pose1*self.pose2)*self.pose3.inverse('pose3')
        assert tri_pose.transformation_matrix() == pytest.approx(np.eye(4))

    def test_pose_mul_vect3(self):
        vec = np.asarray([1.23, 2.3, 1.2])
        new_p = self.pose3*vec
        gt = self.pose3.translation.copy()
        gt[0] += vec[1]
        gt[1] -= vec[0]
        gt[2] += vec[2]
        assert new_p == pytest.approx(gt)

    def test_pose_mul_vect3_1(self):
        vec = np.asarray([[1.23, 2.3, 1.2]])
        new_p = self.pose3*vec.T
        gt = self.pose3.translation.copy()
        gt[0] += vec[0][1]
        gt[1] -= vec[0][0]
        gt[2] += vec[0][2]
        assert new_p == pytest.approx(gt)

    def test_pose_mul_vect4(self):
        vec = np.asarray([1.23, 2.3, 1.2, 1.0])
        new_p = self.pose3*vec
        gt = np.ones((4,))
        gt[0:3] = self.pose3.translation.copy()
        gt[0] += vec[1]
        gt[1] -= vec[0]
        gt[2] += vec[2]
        assert new_p == pytest.approx(gt)

    def test_pose_mul_vect4_1(self):
        vec = np.asarray([[1.23, 2.3, 1.2, 1.0]])
        new_p = self.pose3*vec.T
        gt = np.ones((4, 1))
        gt[0:3] = np.expand_dims(self.pose3.translation.copy(), axis=1)
        gt[0] += vec[0][1]
        gt[1] -= vec[0][0]
        gt[2] += vec[0][2]
        assert new_p == pytest.approx(gt)

    def test_pose_mul_pose(self):
        pose = self.pose1 * self.pose2
        pose_m = np.matmul(self.pose1.transformation_matrix(),
                           self.pose2.transformation_matrix())

        assert pose.transformation_matrix() == pytest.approx(pose_m)


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

        assert merged.positions.values == pytest.approx(np.array([1.0,
                                                                  1.0,
                                                                  2.0,
                                                                  2.0]))

    def test_valid_merge_multiple_instances(self):
        merged = self.joint_trajectory_point1.merge([
            self.joint_trajectory_point2,
            self.joint_trajectory_point4])
        assert merged.joint_set.names == ['joint1',
                                          'joint2',
                                          'joint3',
                                          'joint4',
                                          'joint5',
                                          'joint6']

        assert merged.positions.values == pytest.approx(np.array([1.0,
                                                                  1.0,
                                                                  2.0,
                                                                  2.0,
                                                                  4.0,
                                                                  4.0]))

    def test_non_matching_time(self):
        with pytest.raises(ValueError):
            self.joint_trajectory_point1.merge(
                self.joint_trajectory_point5)

    def test_non_matching_time_multiple_instances(self):
        with pytest.raises(ValueError):
            merged = self.joint_trajectory_point1.merge([
                self.joint_trajectory_point4,
                self.joint_trajectory_point5])

    def test_non_mergeable_joint_sets(self):
        with pytest.raises(ValueError):
            self.joint_trajectory_point1.merge(
                self.joint_trajectory_point3)

    def test_non_mergeable_joint_sets_multiple_instances(self):
        with pytest.raises(ValueError):
            merged = self.joint_trajectory_point1.merge([
                self.joint_trajectory_point2,
                self.joint_trajectory_point3])

    def test_non_mergeable_properties(self):
        with pytest.raises(ValueError):
            self.joint_trajectory_point1.merge(
                self.joint_trajectory_point6)

    def test_non_mergeable_properties_multiple_instances(self):
        with pytest.raises(ValueError):
            merged = self.joint_trajectory_point1.merge([
                self.joint_trajectory_point4,
                self.joint_trajectory_point6])
