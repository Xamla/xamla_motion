# world_view_client.py
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

add_joints_srv_name = '/rosvita/world_view/add_joint_posture'
add_get_joints_srv_name = '/rosvita/world_view/get_joint_posture'
query_joints_posture_srv_name = '/rosvita/world_view/query_joint_postures'
add_pose_srv_name = '/rosvita/world_view/add_pose'
get_pose_srv_name = '/rosvita/world_view/get_pose'
query_poses_srv_name = '/rosvita/world_view/query_poses'
update_joints_srv_name = '/rosvita/world_view/update_joint_posture'
update_poses_srv_name = '/rosvita/world_view/update_pose'
remove_element_srv_name = '/rosvita/world_view/remove_element'
add_folder_srv_name = '/rosvita/world_view/add_folder'

update_cartesian_path_srv_name = '/rosvita/world_view/update_cartesianpath'
set_cartesianpath_srv_name = '/rosvita/world_view/set_cartesianpath'
get_cartesian_path_srv_name = '/rosvita/world_view/get_cartesianpath'
query_cartesian_path_srv_name = '/rosvita/world_view/query_cartesianpath'

set_collision_object_srv_name = '/rosvita/world_view/set_collisionobject'
get_collision_object_srv_name = '/rosvita/world_view/get_collisionobject'
update_collision_object_srv_name = '/rosvita/world_view/update_collisionobject'
query_collison_object_srv_name = '/rosvita/world_view/query_collisionobject'


import rospy


class WorldViewClient(object):

    def __init__(self):
        try:
            pass
        except rospy.ServiceException as exc:
            pass
