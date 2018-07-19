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


class ResourceLock(object):

    def __init__(self, success: bool, lock_id: str,
                 creation_date: str, expiration_date: str):

        self.__success = success
        self.__lock_id = lock_id
        self.__creation_date = creation_date
        self.__expiration_date = expiration_date

    @property
    def success(self):
        """
        success : bool
            If true locking was successful
        """
        return self.__success

    @property
    def lock_id(self):
        """
        lock_id : str
        ID of lock resources necessary to release them later
        """
        return self.__lock_id

    @property
    def creation_date(self):
        """
        creation_date : str
            Date of lock creation
        """
        return self.__creation_date

    @property
    def expiration_date(self):
        """
        expiration_date : str
            Date of lock expiration
        """
        return self.expiration_date


class LeaseBaseLockClient(object):

    def __init__(self):

        self.__resource_lock_service = rospy.ServiceProxy(resource_lock_srv_name,
                                                          QueryLock)

    def _call_lock_service(self):
        pass

    def lock_resources(self):
        pass
