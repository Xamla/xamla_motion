# stepped_motion_state.py
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

from .ik_results import ErrorCodes


class SteppedMotionState(object):
    """
    Class which carries a state snapshot of a stepped motion operation
    """

    def __init__(self, goal_id: str, error_message: str, error_code: int, progress: float):
        """
        Initialize class SteppedMotionState

        Parameters
        ----------
        goal_id : str
            Goal id of the ROS action which is used
            to perform the stepped motion operation
        error_message : str
            String representation of the error
        error_code : int
            Numerical representation if the error
        progress : float 
            Progess of the stepped motion operation 
            in percent range [0.0-1.0]

        Returns
        -------
        SteppedMotionState
            An instance of SteppedMotionState
        """
        self.__goal_id = goal_id
        self.__error_message = ErrorCodes(error_message)
        self.__error_code = error_code
        self.__progress = progress

    @property
    def goal_id(self):
        """
        goal_id : str
            Goal id of the ROS action which is used
            to perform the stepped motion operation
        """
        return self.__goal_id

    @property
    def error_message(self):
        """
        error_message : str
            String representation of the error
        """
        return self.__error_message

    @property
    def error_code(self):
        """
        error_code : xamla_motion.data_types.ErroCodes 
            Numerical representation if the error
        """
        return self.__error_code

    @property
    def progress(self):
        """
        progress : float 
            Progess of the stepped motion operation 
            in percent range [0.0-1.0]
        """
        return self.__progress
