# lock_client.py
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

from ..xamla_motion.xamla_motion_exceptions import ServiceException
from xamlamoveit_msgs.srv import QueryLock, QueryLockRequest
import rospy

resource_lock_srv_name = '/xamlaResourceLockService/query_resource_lock'


class LeaseBaseLockClient(object):

    def __init__(self):
        timeout = rospy.Duration(secs=1)
        try:
            self.__resource_lock_service = rospy.ServiceProxy(resource_lock_srv_name,
                                                              QueryLock)
            self.__resource_lock_service.wait_for_service(timeout)
        except rospy.ServiceException as exc:
            raise ServiceException('Service {} is not available'
                                   ''.format(resource_lock_srv_name)) from exc
