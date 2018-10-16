# example_world_view_client.py
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

from xamla_motion.world_view_client import WorldViewClient
from xamla_motion.data_types import JointValues, JointSet, Pose, CartesianPath
from xamla_motion.data_types import CollisionObject, CollisionPrimitive
from pyquaternion import Quaternion


def main():
    # create entities which should be added to and manipulated in world view
    joint_set1 = JointSet(['Joint1', 'Joint2', 'Joint3'])

    joint_values1 = JointValues(joint_set1, [1.0, 2.0, 3.0])

    joint_set2 = JointSet(['Joint1', 'Joint2'])
    joint_values2 = JointValues(joint_set2, [19.0, 20.0])

    joint_set3 = JointSet(['Joint1', 'Joint4'])
    joint_values3 = JointValues(joint_set3, [100.0, 30.0])

    t1 = [0.502522, 0.2580, 0.3670]
    q1 = Quaternion(w=0.304389, x=0.5272, y=0.68704, z=0.39666)

    t2 = [0.23795, 0.46845, 0.44505]
    q2 = Quaternion(w=0.212097, x=0.470916, y=0.720915, z=0.462096)

    t3 = [0.6089578, 0.3406782, 0.208865]
    q3 = Quaternion(w=0.231852, x=0.33222, y=0.746109, z=0.528387)

    pose1 = Pose(t1, q1, 'world')
    pose2 = Pose(t2, q2, 'world')
    pose3 = Pose(t3, q3, 'world')

    cartesian_path1 = CartesianPath([pose1, pose2, pose3])

    sphere = CollisionPrimitive.create_sphere(0.1, pose2)
    cylinder = CollisionPrimitive.create_cylinder(0.4, 0.05, pose2)
    box = CollisionPrimitive.create_box(0.2, 0.2, 0.1, pose1)

    plane = CollisionPrimitive.create_plane(1.0, 1.0, 1.0, 1.0, pose3)
    cone = CollisionPrimitive.create_cone(0.2, 0.2, pose3)

    collision_object1 = CollisionObject([box])
    collision_object2 = CollisionObject([sphere, cylinder])
    collision_object3 = CollisionObject([plane, cone])

    # create a instance of WorldViewClient to get access to rosvita world view
    world_view_client = WorldViewClient()

    print('---------------- add folder --------------')
    # add folder test at WorldView root
    world_view_client.add_folder('test', '')
    # add folder test at WorldView root/test
    world_view_client.add_folder('joint_values', '/test')

    world_view_client.add_folder('poses', '/test')

    world_view_client.add_folder('cartesian_paths', '/test')

    world_view_client.add_folder('collision_objects', '/test')

    print('---------------- add joint values --------------')
    world_view_client.add_joint_values(
        'joint_values1', 'test/joint_values', joint_values1)
    world_view_client.add_joint_values(
        'joint_values2', 'test/joint_values', joint_values2)
    world_view_client.add_joint_values(
        'joint_values3', 'test/joint_values', joint_values3)
    world_view_client.add_joint_values(
        'test', 'test/joint_values', joint_values3)

    print('---------------- update joint values --------------')
    world_view_client.update_joint_values(
        'joint_values1', 'test/joint_values', joint_values2)
    world_view_client.update_joint_values(
        'joint_values2', 'test/joint_values', joint_values3)
    world_view_client.update_joint_values(
        'joint_values3', 'test/joint_values', joint_values1)

    print('---------------- get joint values --------------')

    get_value = world_view_client.get_joint_values('joint_values1',
                                                   'test/joint_values')
    print('joint_values1 is: ' + str(get_value))

    print('---------------- query joint values --------------')
    # query all values which start with t under test/joint_values
    queried_values = world_view_client.query_joint_values(
        'test/joint_values', 't')

    print('---------------- add poses --------------')
    world_view_client.add_pose(
        'pose1', 'test/poses', pose1)
    world_view_client.add_pose(
        'pose2', 'test/poses', pose2)
    world_view_client.add_pose(
        'pose3', 'test/poses', pose3)
    world_view_client.add_pose(
        'test', 'test/poses', pose3)

    print('---------------- update pose --------------')
    world_view_client.update_pose(
        'pose1', 'test/poses', pose2)
    world_view_client.update_pose(
        'pose2', 'test/poses', pose3)
    world_view_client.update_pose(
        'pose3', 'test/poses', pose1)

    print('---------------- get pose --------------')

    get_value = world_view_client.get_pose('pose1',
                                           'test/poses')
    print('pose1 is: ' + str(get_value))

    print('---------------- query poses --------------')
    # query all poses which start with t under test/pose
    queried_values = world_view_client.query_poses('test/poses', 't')

    print('---------------- add cartesian path --------------')
    world_view_client.add_cartesian_path(
        'cartesian_path1', 'test/cartesian_paths', cartesian_path1)
    world_view_client.add_cartesian_path(
        'test1', 'test/cartesian_paths', cartesian_path1)

    print('---------------- update cartesian path --------------')
    world_view_client.update_cartesian_path(
        'cartesian_path1', 'test/cartesian_paths', cartesian_path1)

    print('---------------- get cartesian path --------------')

    get_value = world_view_client.get_cartesian_path('cartesian_path1',
                                                     'test/cartesian_paths')
    print('cartesian_path1 is: ' + str(get_value))

    print('---------------- query cartesian paths --------------')
    # query all cartesian_paths which start with t under test/cartesian_path
    queried_values = world_view_client.query_cartesian_paths(
        'test/cartesian_paths', 't')

    for v in queried_values:
        print(str(v))

    print('---------------- add collision object --------------')
    world_view_client.add_collision_object(
        'collision_object1', 'test/collision_objects', collision_object1)
    world_view_client.add_collision_object(
        'collision_object2', 'test/collision_objects', collision_object2)
    world_view_client.add_collision_object(
        'collision_object3', 'test/collision_objects', collision_object3)
    world_view_client.add_collision_object(
        'test1', 'test/collision_objects', collision_object1)

    print('---------------- update collision object --------------')
    world_view_client.update_collision_object('collision_object1',
                                              'test/collision_objects',
                                              collision_object1)

    print('---------------- get collision object --------------')

    get_value = world_view_client.get_collision_object('collision_object1',
                                                       'test/collision_objects')
    print('collision_object1 is: ' + str(get_value))

    print('---------------- query collision object --------------')
    # query all collision_objects which start with t under test/collision_object
    queried_values = world_view_client.query_collision_objects(
        'test/collision_objects', 't')

    for v in queried_values:
        print(str(v))

    input('Press enter to clean up')

    print('----------------- remove folder test ---------------')
    world_view_client.remove_element('test', '')


if __name__ == '__main__':
    main()
