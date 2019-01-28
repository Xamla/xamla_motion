import asyncio
import pathlib
import pickle
import time
import gzip
import io
import gc

import numpy as np
import ptvsd
import rospy
from pyquaternion import Quaternion
from . import EndEffector, WorldViewClient
from.data_types import Pose
from .utility import register_asyncio_shutdown_handler
from xamlamoveit_msgs.srv import SetJointPosture, SetJointPostureRequest

from .trajectory_caching import (SampleRectangle,
                                             TaskTrajectoryCache,
                                             create_trajectory_cache,
                                             move_with_trajectory_cache)

# ptvsd.wait_for_attach()


async def try_move_with_cache(target: Pose,
                              trajectory_cache: TaskTrajectoryCache,
                              end_effector: EndEffector,
                              client: WorldViewClient):

    start = end_effector.get_current_pose()

    client.add_pose('target', '/trajectory_caching', target)

    cached_target = await move_with_trajectory_cache(cache=trajectory_cache,
                                                     end_effector=end_effector,
                                                     start=None,
                                                     target=target,
                                                     max_position_diff_radius=0.02)

    client.add_pose('nearest_cached', '/trajectory_caching', cached_target)

    await end_effector.move_poses(target)

    await end_effector.move_poses(start)


def main():
    client = WorldViewClient()
    try:
        client.remove_element('trajectory_caching', '/')        
    except Exception as e:

        print(e)
    client.add_folder('trajectory_caching', '/')
        
    # get robot initial state and create ros service handler to set it
    new_robot_state = client.get_joint_values('initial_posture_full_body',
                                              'Stations/Handling/PullOverRight2Left')
    set_state_service_handle = rospy.ServiceProxy('/sda10d/xamlaSda10dController/set_state',
                                                  SetJointPosture)

    request = SetJointPostureRequest()
    request.joint_names = new_robot_state.joint_set.names
    request.point.positions = new_robot_state.values

    response = set_state_service_handle(request)

    if not response.success:
        raise RuntimeError('set robot state was not successful')

    time.sleep(1)

    # create end effector, seed joint values and start pose
    left_arm = EndEffector.from_end_effector_name('EE_manipulator_left_torso')

    left_arm.move_group.collision_check = True
    left_arm.move_group.velocity_scaling = 1.0

    seed = left_arm.move_group.get_current_joint_positions()

    start_pose = left_arm.compute_pose(seed)

    # create target area
    handling_station_left_top = client.get_pose('PickPlane',
                                                'ClickAndPick/Debug/')

    # handling_station = handling_station_left_top.translate([0.0, 0.0, +0.04])
    mid_transform = Pose.from_transformation_matrix(np.eye(4))
    mid_transform = mid_transform.translate([0.13, 0.085, -0.04])
    handling_station = handling_station_left_top * mid_transform

    client.add_pose('mid_point', 'trajectory_caching', handling_station)

    path = pathlib.Path(__file__).parent
    pickle_path = pathlib.Path(
        path/'handling_station_left_arm_pre_pick_cache_fast.pickle')

    if not pickle_path.exists():

        # define for which target pose quaternions to create trajectories
        quaternions = []

        for i in range(7):
            a = (np.pi/4)*i
            q = Quaternion(matrix=np.array([[np.cos(a), -np.sin(a), 0],
                                            [np.sin(a), np.cos(a), 0],
                                            [0, 0, 1]], dtype=float))

            r = handling_station.quaternion * q

            quaternions.append(r)

        sample_rect = SampleRectangle(origin=handling_station,
                                      size=[0.4, 0.25, 0.0],
                                      resolution=[0.012, 0.012, 0.012],
                                      quaternions=quaternions)

        trajectory_cache = create_trajectory_cache(end_effector=left_arm,
                                                   seed=new_robot_state,
                                                   start=start_pose,
                                                   target=sample_rect)

        gc.disable()
        with gzip.open(str(pickle_path), 'wb') as handle:
            pickle.dump(trajectory_cache, io.BufferedWriter(handle),
                        protocol=pickle.HIGHEST_PROTOCOL)
        gc.enable()
    else:
        gc.disable()
        with gzip.open(str(pickle_path), 'rb') as handle:
            trajectory_cache = pickle.load(io.BufferedReader(handle))
        gc.enable()

    a = (np.pi/3)*0
    quaternion = Quaternion(matrix=np.array([[np.cos(a), -np.sin(a), 0],
                                             [np.sin(a), np.cos(a), 0],
                                             [0, 0, 1]], dtype=float))

    v = handling_station.translation.copy()
    v[0:2] += np.random.uniform(-0.02, 0.02, 2)
    target = Pose(v, handling_station.quaternion*quaternion)

    loop = asyncio.get_event_loop()
    register_asyncio_shutdown_handler(loop)

    try:
        loop.run_until_complete(try_move_with_cache(target,
                                                    trajectory_cache,
                                                    left_arm,
                                                    client))
    finally:
        loop.close()

    input('press enter to detele the added world view content')
    client.remove_element('trajectory_caching', '/')


if __name__ == '__main__':
    main()
