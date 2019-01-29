# xamla_motion_exceptions.py
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


class XamlaMotionException(Exception):
    """
    Base xamla_motion exception to enable lib depended 
    exception handling
    """

    def __init__(self, msg, original_exception=None):
        super(XamlaMotionException, self).__init__(msg)
        self.original_exception = original_exception


class ServiceException(XamlaMotionException):
    """
    Underlying service is not available or
    call unsuccessful (e.g. ROS services)
    """

    def __init__(self, msg, error_code=None, original_exception=None):
        super(ServiceException, self).__init__(msg)
        self.original_exception = original_exception
        self.error_code = error_code


class ArgumentError(XamlaMotionException):
    """
    Wrong format or type of argument
    """

    def __init__(self, msg, original_exception=None):
        super(ArgumentError, self).__init__(msg)
        self.original_exception = original_exception
