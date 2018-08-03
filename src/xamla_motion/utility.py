# utility.py
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

from typing import List
from collections import Iterable

from .xamla_motion_exceptions import ServiceException
from xamlamoveit_msgs.srv import QueryLock, QueryLockRequest
import rospy
import re

resource_lock_srv_name = '/xamlaResourceLockService/query_resource_lock'


class ResourceLock(object):

    def __init__(self, success: bool, resource_ids: List[str], lock_id: str,
                 creation_date: str, expiration_date: str):

        self.__success = success
        self.__resource_ids = resource_ids
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
    def resource_ids(self):
        """
        resource_ids : str
            ID of lock resources necessary to release them later
        """
        return self.__resource_ids

    @property
    def lock_id(self):
        """
        lock_id : str
            UUID which identifies the client who request the lock
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


class LeaseBaseLock(object):
    """
    Class to lock resource 

    Parameters
    ----------
    resource_ids : Iterable[str]
        Ids of the resources where lock them is requested

    Returns
    -------
    LeaseBaseLock
        Instance of LeaseBaseLock

    Raises
    ------
    TypeError
        If resource_ids is not Iterable of str
    ServiceError
        If lock service is no available or finish unsuccessfully

    Examples
    --------
    The lock and release can be implemented comfortably with 
    python's with statement
    >>> with LeaseBaseLock(resource_ids) as lock:
    >>>     do something with resources
    """

    def __init__(self, resource_ids: List[str], lock_id: str=''):

        self.__request = QueryLockRequest()
        self.__request.id_resources = list(resources_ids)
        self.__request.id_lock = str(lock_id)

        self.__lock_service = rospy.ServiceProxy(resource_lock_srv_name,
                                                 QueryLock)

        self.__resource_lock = None

    def __enter__(self):
        self._call_lock_service(release=False)

    def __exit__(self):
        self._call_lock_service(release=True)

    @property
    def resource_lock(self):
        """
        resource_lock: ResourceLock or None
            Currently locked resources
        """
        return self.__resource_lock

    def _call_lock_service(self, release=False):
        self.__request.release = release

        try:
            response = self.__lock_service.call(self.__request)
        except rospy.ServiceException as exc:
            raise ServiceException('lock service is not available') from exc

        if not response.success:
            raise ServiceException('lock service finish not succuessfully'
                                   ', reason: {}'.format(response.error_msg))

        self.__resource_lock = ResourceLock(resonse.success, response.id_resources,
                                            response.id_lock, response.creation_date,
                                            response.expiration_date)


class ROSNodeSteward(object):
    """
    Maintain ros node

    If no rosnode exist create a new rosnode 
    and handle shutdown else keep the existing one
    """
    __is_ros_init = 0
    __self_created = False

    def __init__(self):
        if (not __is_ros_init and
                re.sub('[^A-Za-z0-9]+', '', rospy.get_name()) == 'unnamed'):
            rospy.init_node('xamla_motion',
                            anonymous=True,
                            disable_signals=True)
            __self_created = True

        __is_ros_init += 1

    def __del__(self):
        __is_ros_init -= 1
        if not __is_ros_init and __self_created:
            rospy.signal_shutdown('xamla_motion shutdown')
