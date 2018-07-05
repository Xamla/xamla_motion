# ik_results.py
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

#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function)  # , unicode_literals)
from future.builtins import map
from future.utils import raise_from, raise_with_traceback

from moveit_msgs.msg import MoveItErrorCodes


class IkResults(object):

    def __init__(self, path, error_codes):
        self.__path = path
        self.__error_codes = error_codes

    @property
    def path(self):
        """
        path : JointPath
            solutions from the inverse kinematics
        """
        return self.__path

    @property
    def error_codes(self):
        """
        error_code : List[MoveItErrorCodes]
            error codes
        """
        return self.__error_codes

    @property
    def succeeded(self):
        if all([e == MoveItErrorCodes.SUCCESS
                for e in self.__error_codes]):
            return True
        else:
            return False

    def __str__(self):
        return str(error_codes)

    def __repr__(self):
        return self.__str__()
