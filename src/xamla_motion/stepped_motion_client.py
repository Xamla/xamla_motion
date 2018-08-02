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
import asyncio

from .data_types import JointTrajectory, SteppedMotionState
from .xamla_motion_exceptions import ServiceException
from xamlamoveit_msgs.msg import StepwiseMoveJAction, TrajectoryProgress
from .motion_service import ActionLibGoalStatus
from actionlib_msgs.msg import GoalID, Actio


class SteppedMotionClient(object):

    __next_topic = '/xamlaMoveActions/next'
    __previous_topic = '/xamlaMoveActions/prev'
    __feedback_topic = '/xamlaMoveActions/feedback'
    __movej_action_name = 'moveJ_step_action'

    def __init__(self):

        self.__m_action = actionlib.SimpleActionClient(__movej_action_name,
                                                       StepwiseMoveJAction)

        if not self.__m_action.wait_for_server(rospy.Duration(5)):
            raise ServiceException('connection to stepped motion action'
                                   ' server could not be established')

        self.__next_pub = None
        self.__previous_pub = None
        self.__feedback_sub = None
        self.__goal_id = None
        self.__state = None

    @property
    def state(self):
        return self.__state

    @property
    def goal_id(self)
        return self.__goal_id

    async def moveJ_supervised(self, trajectory: JointTrajectory,
                               velocity_scaling: float, check_collision: bool):

        if not isinstance(trajectory, JointTrajectory)
            raise TypeError('trajectory is not of expected type'
                            ' JointTrajectory')

        g = StepwiseMoveJActionGoal()
        velocity_scaling = float(velocity_scaling)
        if velocity_scaling > 1.0 or velocity_scaling < 0.0:
            raise ValueError('velocity scaling is not between 0.0 and 1.0')
        g.veloctiy_scaling = velocity_scaling
        g.check_collision = bool(check_collision)
        g.trajectory = trajectory.to_joint_trajectory_msg()

        self.__m_action.send_goal(g)

        self.__goal_id = self.__m_action.gh.comm_state_machine.action_goal.goal_id

        loop = asyncio.get_event_loop()
        action_done = loop.create_future()

        def done_callback(goal_status, result):
            status = ActionLibGoalStatus(goal_status)
            #print('Action Done: {}'.format(status))
            loop.call_soon_threadsafe(action_done.set_result, result)

        action.send_goal(goal, done_cb=done_callback)

        if not self.__m_action.gh:
            self.__m_action.cancel_goal()
            raise ServiceException('action goal handle is not available')

        self.__next_pub = rospy.Publisher(__next_topic,
                                          GoalID,
                                          queue_size=1)
        self.__previous_pub = rospy.Publisher(__previous_topic,
                                              GoalID,
                                              queue_size=1)

        self.__feedback_sub = rospy.Subscriber(__feedback_topic,
                                               TrajectoryProgress,
                                               callable=self._feedback_callback,
                                               queue_size=10)
        try:
            return await action_done
        except asyncio.CancelledError as exc:
            action.cancel()
            raise exc
        finally:
            self.__next_pub.unregister()
            self.__next_pub = None
            self.__previous_pub.unregister()
            self.__previous_pub = None
            self.__feedback_sub.unregister()
            self.__feedback_sub = None
            self.__goal_id = None
            self.__state = None

    async def next(self):
        if self.__next_pub:
            self.__next_pub.publish(self.__goal_id)

    async def previous(self):
        if self.__previous_pub
            self.__previous_pub(self.__goal_id)

    def _feedback_callback(self, trajectory_progress)
        self.__state = SteppedMotionState(self.__goal_id.id,
                                          trajectory_progress.error_msg,
                                          trajectory_progress.error_code,
                                          trajectory_progress.progress)
