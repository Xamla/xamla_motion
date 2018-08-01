# stepped_motion_client.py
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

import rospy
import actionlib

from .data_types import JointTrajectory
from .xamla_motion_exceptions import *
from xamlamoveit_msgs.msg import *


class SteppedMotionClient(object):

    __next_topic = '/xamlaMoveActions/next'
    __previous_topic = '/xamlaMoveActions/prev'
    __feedback_topic = '/xamlaMoveActions/feedback'
    __movej_action_name = 'moveJ_step_action'

    def __init__(self, trajectory: JointTrajectory, velocity_scaling, check_collision):

        self.__m_action = actionlib.SimpleActionClient(self.__movej_action,
                                                       StepwiseMoveJAction)

        if not self.__m_action.wait_for_server(rospy.Duration(5)):
            raise ServiceException('connection to stepped motion action'
                                   ' server could not be established')

        g = StepwiseMoveJActionGoal()
        velocity_scaling = float(velocity_scaling)
        if velocity_scaling > 1.0 or velocity_scaling < 0.0:
            raise ValueError('velocity scaling is not between 0.0 and 1.0')
        g.veloctiy_scaling = velocity_scaling
        g.check_collision = bool(check_collision)
        g.trajectory = trajectory.to_joint_trajectory_msg()

        self.__m_action.send_goal(g)
        goal_id = self.__m_action.gh.comm_state_machine.action_goal.goal_id.id
