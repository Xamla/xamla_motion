import datetime
import time
import enum
from abc import ABC, abstractmethod
from typing import Callable, Dict, Iterable, Union

import numpy as np
import rospy
from pyquaternion import Quaternion
from sklearn.neighbors import BallTree
from xamlamoveit_msgs.srv import SetJointPosture, SetJointPostureRequest

from .motion_client import EndEffector, MoveGroup
from .data_types import (CartesianPath, JointPath, JointTrajectory,
                         JointValues, Pose)


class SampleVolume(ABC):

    """
    This class defines a sample area.
    """

    def __init__(self, origin: Pose, size: Iterable[float],
                 resolution: Iterable[float],
                 quaternions: Iterable[Quaternion]):
        self._origin = origin
        self._size = size
        self._resolution = resolution
        self._quaternions = quaternions

        self._check_input()
        self._sample_positions = self._generate_samples()

    @property
    def origin(self):
        return self._origin

    @property
    def quaternions(self):
        return self._quaternions

    @property
    def sample_positions(self):
        return self._sample_positions

    @abstractmethod
    def _check_input(self):
        pass

    @abstractmethod
    def _generate_samples(self):
        pass


class SampleBox(SampleVolume):

    """
    This class defines an area where positions are sampled
    defined by the parameters.
    """

    def __init__(self, origin: Pose, size: Iterable[float],
                 resolution: Iterable[float], quaternions: Iterable[Quaternion]) -> np.ndarray:
        """
        Sample poses

        Parameters
        ----------
        origin : Pose
            The origin of the rectangle as a midpoint
        size : Iterable[float]
            The size if the rectangle
            Must be 3-dimensional
        resolution : Iterable[float]
            The
        quaternions: Iterable[Quaternion]
            A set of quaternions

        """

        super(SampleBox, self).__init__(
            origin, size, resolution, quaternions)

    def _check_input(self):
        pass

    def _generate_samples(self):
        x_ss = self._size[0]/2.0
        y_ss = self._size[1]/2.0
        z_ss = self._size[2]/2.0

        eps = np.finfo(float).eps

        xyz = np.mgrid[-x_ss:x_ss+eps:self._resolution[0],
                       -y_ss:y_ss+eps:self._resolution[1],
                       -z_ss:z_ss+eps:self._resolution[2]]

        sample_positions = np.dot(self._origin.transformation_matrix(),
                                  np.vstack((xyz[0].ravel(),
                                             xyz[1].ravel(),
                                             xyz[2].ravel(),
                                             np.ones(xyz[0].size)))
                                  )[0:3, :]

        return sample_positions


@enum.unique
class TrajectoryCacheType(enum.Enum):
    ONETOONE = 0,
    ONETOMANY = 1,
    MANYTOONE = 2,
    MANYTOMANY = 3


class TaskTrajectoryCache(object):

    """
    The TaskTrajectoryCache manages a set of trajectories.

    """

    def __init__(self, start: Union[Pose, Iterable],
                 target: Union[Pose, Iterable],
                 trajectory: Union[JointTrajectory, Iterable],
                 start_ball_tree: BallTree,
                 target_ball_tree: BallTree,
                 end_effector_name: str,
                 cache_type: TrajectoryCacheType):

        if isinstance(start, Iterable):
            self._start = tuple(start)
        else:
            self._start = start

        if isinstance(target, Iterable):
            self._target = tuple(target)
        else:
            self._target = target

        if isinstance(trajectory, Iterable):
            self._trajectory = tuple(trajectory)
        else:
            self._trajectory = trajectory

        self._start_ball_tree = start_ball_tree
        self._target_ball_tree = target_ball_tree
        self._end_effector_name = end_effector_name
        self._cache_type = cache_type

    @property
    def cache_type(self):
        return self._cache_type

    @property
    def end_effector_name(self):
        return self._end_effector_name

    def _get_trajectory_with_nearest_rotation(self, request,
                                              rotation_cache_pos,
                                              rotation_cache_trajectory):
        poses = rotation_cache_pos
        trajectories = rotation_cache_trajectory

        min_diff = np.inf
        rot_index = -1

        for i, pose in enumerate(poses):
            diff = Quaternion.absolute_distance(request.quaternion,
                                                pose.quaternion)

            if diff < min_diff:
                min_diff = diff
                rot_index = i

        return trajectories[rot_index], poses[rot_index]

    def get_trajectory(self, start: Pose, target: Pose,
                       max_position_diff_radius: float):

        if self._cache_type == TrajectoryCacheType.ONETOONE:

            if start != self._start:
                raise RuntimeError('requested start {} and cached start {}'
                                   ' are not equal'.format(start,
                                                           self._start))

            if target != self._target:
                raise RuntimeError('requested target {} and cached target {}'
                                   ' are not equal'.format(target,
                                                           self._target))

            start_pose = start
            target_pose = target
            start_joint_values = self._trajectory[0].positions
            target_joint_values = self._trajectory[-1].positions

            return (self._trajectory, start_pose, target_pose,
                    start_joint_values, target_joint_values)

        elif self._cache_type == TrajectoryCacheType.ONETOMANY:
            if start != self._start:
                raise RuntimeError('requested start {} and cached start {}'
                                   ' are not equal'.format(start,
                                                           self._start))

            qv = np.expand_dims(target.translation, 0)
            dist, index = self._target_ball_tree.query(qv)
            index = int(index)

            if float(dist) > max_position_diff_radius:
                raise RuntimeError('distance {} between nearest cached trajectory'
                                   ' end point and requested target is greater than'
                                   ' defined max difference {}'.format(float(dist),
                                                                       max_position_diff_radius))

            poses = self._target[index]
            trajectories = self._trajectory[index]

            cached_trajectory, cached_target = self._get_trajectory_with_nearest_rotation(target,
                                                                                          poses,
                                                                                          trajectories)

            start_pose = start
            target_pose = cached_target
            start_joint_values = cached_trajectory[0].positions
            target_joint_values = cached_trajectory[-1].positions

            return (cached_trajectory, start_pose, target_pose,
                    start_joint_values, target_joint_values)

        elif self._cache_type == TrajectoryCacheType.MANYTOONE:
            if target != self._target:
                raise RuntimeError('requested target {} and cached target {}'
                                   ' are not equal'.format(target,
                                                           self._target))

            qv = np.expand_dims(start.translation, 0)
            dist, index = self._start_ball_tree.query(qv)
            index = int(index)

            if float(dist) > max_position_diff_radius:
                raise RuntimeError('distance {} between nearest cached trajectory'
                                   ' start point and requested start is greater than'
                                   ' defined max difference {}'.format(float(dist),
                                                                       max_position_diff_radius))

            poses = self._start[index]
            trajectories = self._trajectory[index]

            cached_trajectory, cached_start = self._get_trajectory_with_nearest_rotation(start,
                                                                                         poses,
                                                                                         trajectories)

            start_pose = cached_start
            target_pose = target
            start_joint_values = cached_trajectory[0].positions
            target_joint_values = cached_trajectory[-1].positions

            return (cached_trajectory, start_pose, target_pose,
                    start_joint_values, target_joint_values)

        else:
            raise NotImplementedError('many to many cached trajectories are '
                                      'currently not supported')

    def to_dict(self):
        return vars(self)


def _generate_trajectory(start: Pose, target: Pose,
                         end_effector: EndEffector,
                         seed: JointValues):

    cartesian_path = CartesianPath([start, target])

    collision_check = end_effector.move_group.collision_check

    result = end_effector.inverse_kinematics_many(poses=cartesian_path,
                                                  collision_check=collision_check,
                                                  seed=seed,
                                                  timeout=datetime.timedelta(
                                                      seconds=5),
                                                  const_seed=False)

    if not result.succeeded:
        raise RuntimeError('ik not succeeded: {}'.format(result.error_codes))

    path = result.path

    trajectory, _ = end_effector.move_group.plan_move_joints(path)

    return trajectory


def _set_robot_state(pose, end_effector, seed):
    new_robot_state = end_effector.inverse_kinematics(pose=pose,
                                                      collision_check=True,
                                                      seed=seed,
                                                      timeout=datetime.timedelta(
                                                          seconds=5),
                                                      const_seed=False)

    set_state_service_handle = rospy.ServiceProxy('/sda10d/xamlaSda10dController/set_state',
                                                  SetJointPosture)

    request = SetJointPostureRequest()
    request.joint_names = new_robot_state.joint_set.names
    request.point.positions = new_robot_state.values

    response = set_state_service_handle(request)

    if not response.success:
        raise RuntimeError('set robot state was not successful')

    time.sleep(0.2)


def create_trajectory_cache(end_effector: EndEffector,
                            seed: JointValues,
                            start: Union[Pose, SampleVolume],
                            target: Union[Pose, SampleVolume]) -> TaskTrajectoryCache:
    """
    Factory function to create a TaskTrajectoryCache instance

    Parameters
    ----------
    end_effector : EndEffector
        The end effector being used
    seed : JointValues
        A seed being a template for the trajectories
    start : Union[Pose, SampleVolume]
        A Pose or a SampleVolume, defining the start of the trajectory(/ies)
    target: Union[Pose, SampleVolume]
        A Pose or a SampleVolume, defining the end of the trajectory(/ies)

    Returns
        -------
    TaskTrajectoryCache:
        The trajectory cache created.
    """

    if isinstance(start, SampleVolume) and isinstance(target, SampleVolume):
        raise NotImplementedError('start and target as areas is'
                                  ' currently not supported')
    elif isinstance(start, SampleVolume) and isinstance(target, Pose):
        # MANYTOONE
        starts = []
        executables = []
        excludes = []
        lenght = start.sample_positions.shape[1]
        for i, v in enumerate(start.sample_positions.T):
            print('generate trajectories for position {}'
                  ' of {} positions'.format(i+1, lenght))
            poses = []
            trajectories = []
            try:
                for a in start.quaternions:
                    pose = Pose(v, a)
                    _set_robot_state(pose, end_effector, seed)
                    trajectories.append(_generate_trajectory(pose, target,
                                                             end_effector,
                                                             seed))
                    poses.append(pose)
            except Exception as exc:
                print('remove start position: {} because of {}'.format(pose.translation,
                                                                       exc))
                excludes.append(i)
                continue

            starts.append(tuple(poses))
            executables.append(tuple(trajectories))

        valid_position = np.in1d(np.arange(len(start.sample_positions.T)),
                                 excludes, invert=True)
        start_ball_tree = BallTree(
            start.sample_positions.T[valid_position, :])

        return TaskTrajectoryCache(start=starts,
                                   target=target,
                                   trajectory=executables,
                                   start_ball_tree=start_ball_tree,
                                   target_ball_tree=None,
                                   end_effector_name=end_effector.name,
                                   cache_type=TrajectoryCacheType.MANYTOONE)

    elif isinstance(target, SampleVolume) and isinstance(start, Pose):
        # ONETOMANY
        _set_robot_state(start, end_effector, seed)

        targets = []
        executables = []
        excludes = []
        lenght = target.sample_positions.shape[1]
        for i, v in enumerate(target.sample_positions.T):
            print('generate trajectories for position {}'
                  ' of {} positions'.format(i+1, lenght))
            poses = []
            trajectories = []
            try:
                for a in target.quaternions:
                    pose = Pose(v, a)
                    trajectories.append(_generate_trajectory(start, pose,
                                                             end_effector,
                                                             seed))
                    poses.append(pose)
            except Exception as exc:
                print('remove target position: {} because of {}'.format(pose.translation,
                                                                        exc))
                excludes.append(i)
                continue
            targets.append(tuple(poses))
            executables.append(tuple(trajectories))

        valid_position = np.in1d(np.arange(len(target.sample_positions.T)),
                                 excludes, invert=True)
        target_ball_tree = BallTree(
            target.sample_positions.T[valid_position, :])

        return TaskTrajectoryCache(start=start,
                                   target=targets,
                                   trajectory=executables,
                                   start_ball_tree=None,
                                   target_ball_tree=target_ball_tree,
                                   end_effector_name=end_effector.name,
                                   cache_type=TrajectoryCacheType.ONETOMANY)
    else:
        trajectory = _generate_trajectory(start, target,
                                          end_effector,
                                          seed)

        return TaskTrajectoryCache(start=start,
                                   target=target,
                                   trajectory=trajectory,
                                   start_ball_tree=None,
                                   target_ball_tree=None,
                                   end_effector_name=end_effector.name,
                                   cache_type=TrajectoryCacheType.ONETOONE)


async def move_with_trajectory_cache(cache: TaskTrajectoryCache, end_effector: EndEffector,
                                     start_joint_values: Union[None, JointValues], target_pose: Pose,
                                     max_position_diff_radius: float, logger=None,
                                     collision_check: bool=True):

    move_group = end_effector.move_group

    if end_effector.name != cache.end_effector_name:
        try:
            logger.error('provided trajectory cache not contains trajectories'
                         ' for end effector: {}'.format(end_effector.name))
        except:
            pass

        raise RuntimeError('provided trajectory cache not contains trajectories'
                           ' for end effector: {}'.format(end_effector.name))

    if start_joint_values is None:
        try:
            logger.error('move with cache: start is None use current'
                         ' end effector: {} pose as start pose'.format(end_effector.name))
        except:
            pass

        start_joint_values = move_group.get_current_joint_positions()

    start_pose = end_effector.compute_pose(start_joint_values)

    (cached_trajectory,
     cached_start_pose,
     cached_target_pose,
     cached_start_joint_values,
     cached_target_joint_values) = cache.get_trajectory(start_pose, target_pose,
                                                        max_position_diff_radius)

    if cache.cache_type == TrajectoryCacheType.MANYTOONE:
        await move_group.move_joints(JointPath(cached_start_joint_values.joint_set,
                                               [start_joint_values, cached_start_joint_values]),
                                     collision_check=collision_check)

    services = end_effector.motion_service

    await services.execute_joint_trajectory(cached_trajectory, collision_check)

    return cached_start_pose, cached_target_pose, cached_start_joint_values, cached_target_joint_values
