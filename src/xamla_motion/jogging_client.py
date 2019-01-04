# jogging_client.py
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

import rospy
import actionlib
#from datetime import timedelta
import enum

from xamla_motion.data_types import Pose, JointValues, Twist

from geometry_msgs.msg import PoseStamped, TwistStamped
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from xamlamoveit_msgs.msg import ControllerState 
from xamlamoveit_msgs.srv import GetSelected, SetString
from xamlamoveit_msgs.srv import GetFloat, SetFloat 
from xamlamoveit_msgs.srv import GetFlag, SetFlag 
from xamlamoveit_msgs.srv import StatusController

from std_srvs.srv import SetBool

from xamla_motion.utility import ROSNodeSteward
from xamla_motion.xamla_motion_exceptions.exceptions import ServiceException

@enum.unique
class JoggingErrorCode(enum.Enum):
    OK = 1
    INVALID_IK = -1
    SELF_COLLISION = -2
    SCENE_COLLISION = -3
    FRAME_TRANSFORM_FAILURE = -4
    IK_JUMP_DETECTED = -5
    CLOSE_TO_SINGULARITY = -6
    JOINT_LIMITS_VIOLATED = -7
    INVALID_LINK_NAME = -8
    TASK_SPACE_JUMP_DETECTED = -9


class JoggingClientFeedbackState():
    """ A data structure class representing the current jogging client state """

    def __init__(self, joint_distance = None,
                    cartesian_distance = None,
                    error_code = None,
                    converged = None,
                    self_collision_check_enabled = None,
                    joint_limits_check_enabled = None,
                    scene_collision_check_enabled = None
                ):
        # init all member variables
        self._joint_distance = joint_distance
        self._cartesian_distance = cartesian_distance
        self._error_code = error_code
        self._converged = converged
        self._self_collision_check_enabled = self_collision_check_enabled
        self._joint_limits_check_enabled = joint_limits_check_enabled
        self._scene_collision_check_enabled = scene_collision_check_enabled

    @property
    def joint_distance(self): 
        """
        joint_distance : List[float] (read only)
            The distance of the joints [m]
        """
        return self._joint_distance

    @property
    def cartesian_distance(self): 
        """
        cartesian_distance : List[float] (read only)
            The cartesian Distance [m]
        """
        return self._cartesian_distance

    @property
    def error_code(self): 
        """
        error_code : JoggingErrorCode (read only)
            The current error code
        """
        return self._error_code

    @property
    def converged(self): 
        """
        converged : bool (read only)
            True when converged
        """
        return self._converged

    @property
    def self_collision_check_enabled(self): 
        """
        self_collision_check_enabled : bool (read only)
            True if check for self-collision is enabled
        """
        return self._self_collision_check_enabled

    @property
    def joint_limits_check_enabled(self): 
        """
        joint_limits_check_enabled : bool (read only)
            True if check for joint limits is enabled
        """
        return self._joint_limits_check_enabled

    @property
    def scene_collision_check_enabled(self): 
        """
        scene_collision_check_enabled : bool (read only)
            True if check for scene collision is enabled
        """
        return self._scene_collision_check_enabled

    def __str__(self):
        ret_string = "JoggingClientFeedbackState: \n"
        ret_string += "joint_distance: {}\n".format(self._joint_distance) 
        ret_string += "cartesian_distance: {}\n".format(self._cartesian_distance) 
        ret_string += "error_code: {}\n".format(self._error_code) 
        ret_string += "converged: {}\n".format(self._converged) 
        ret_string += "self_collision_check_enabled: {}\n".format(self._self_collision_check_enabled) 
        ret_string += "joint_limits_check_enabled: {}\n".format(self._joint_limits_check_enabled) 
        ret_string += "scene_collision_check_enabled: {}\n".format(self._scene_collision_check_enabled) 
        return ret_string


class JoggingClientFeedbackEvent(object):
    """
    A simple observer which registers callback functions
    TODO: Could be improved by accepting object instead of functions, or using an external library altogether

    Methods
    -------
    register(callback_function)
        Registers a callback function to be called when state updates
    unregister(callback_function)
        Unregisters a callback function 
    """

    def __init__(self):
        self._subscribers = set()

    def register(self, callback_function) -> None:
        """
        Register a callback function for the feedback
        
        Parameters
        ----------
        callback_function : function
            Function which takes an instance of JoggingClientFeedbackState
        """
        self._subscribers.add(callback_function)

    def unregister(self, callback_function) -> None:
        """
        Unregisters a callback function 
        
        Parameters
        ----------
        callback_function : function
            Function which takes an instance of JoggingClientFeedbackState and has been registered
        """
        self._subscribers.discard(callback_function)

    def _dispatch(self, state: JoggingClientFeedbackState):
        for callback_function in self._subscribers:
            # ignore subscribers for which the call_back function crashes
            try:
                callback_function(state)
            except Exception as e:
                print("Ignoring callback function {}".format(callback_function))
                print(e)
                


class JoggingClient(JoggingClientFeedbackEvent):
    """ 
    A jogging client

    TODO: One could argue that the feedback event should rather be used by composition than inheritance
    
    Methods
    -------
    send_set_point(setPoint)
        Jogging to Pose
    send_send_velocities(velocities)
        Jogging by applying joint velocities
    send_twist(twist)
        Jogging by applying a twist 
    get_velocity_scaling()
        Get velocity scaling
    set_velocity_scaling(value)
        Set velocity scaling
    get_move_group_name()
        TODO: verify this
        Get a List of all move groups
    set_move_group_name(name)
        Set the name of the current move group used for jogging
    get_endeffector_name()
        Get a list of the names of all endeffectors
    set_endeffector_name(name)
        Set the name of the current endeffector used for jogging
    get_flag(name)
        Get flag of name "name"
    set_flag(name, value)
        Set flag of name "name"
    toggle_tracking(toggle)
        Toggle tracking on or off
    start():
        Toggle tracking on
    stop():
        Toggle tracking off
    """

    __setpoint_topic = "/xamlaJointJogging/jogging_setpoint"
    __jogging_command_topic = "/xamlaJointJogging/jogging_command"
    __jogging_twist_topic = "/xamlaJointJogging/jogging_twist"
    __jogging_feedback_topic = "/xamlaJointJogging/feedback"

    __toggle_tracking_service_id = "/xamlaJointJogging/start_stop_tracking"
    __get_move_group_name_service_id = "/xamlaJointJogging/get_movegroup_name"
    __set_move_group_name_service_id = "/xamlaJointJogging/set_movegroup_name"
    __get_endeffector_name_service_id = "/xamlaJointJogging/get_endeffector_name"
    __set_endeffector_name_service_id = "/xamlaJointJogging/set_endeffector_name"
    __status_service_id = "xamlaJointJogging/status"
    __get_velocity_scaling_service_id = "/xamlaJointJogging/get_velocity_scaling"
    __set_velocity_scaling_service_id = "/xamlaJointJogging/set_velocity_scaling"
    __get_flag_service_id= "xamlaJointJogging/get_flag"
    __set_flag_service_id= "xamlaJointJogging/set_flag"

    def __init__(self):
        super(JoggingClient, self).__init__()
        self.__ros_node_steward = ROSNodeSteward()
        self._jogging_event = JoggingClientFeedbackEvent
        self._init_topics()
        self._init_services()

    def _init_topics(self):
        self._set_point_pub = rospy.Publisher(self.__setpoint_topic, 
                                        PoseStamped,
                                        queue_size=5)
        self._jogging_command_pub = rospy.Publisher(self.__jogging_command_topic, 
                                            JointTrajectory, 
                                            queue_size=5)
        self._jogging_twist_pub = rospy.Publisher(self.__jogging_twist_topic, 
                                            TwistStamped, 
                                            queue_size=5)
        self.__feedback_sub = rospy.Subscriber(self.__jogging_feedback_topic,
                                            ControllerState,
                                            callback=self._handle_jogging_feedback,
                                            queue_size=1)

    def _init_services(self):
        
        def exc_wrap_call(name, msg_type):
            """ utility function for dry purpose

            Call rospy.ServiceProxy for given name and msg type
            """
            try:
                return rospy.ServiceProxy(name, msg_type)
            except rospy.ServiceException as exc:
                raise ServiceException('connection for service with name: ' +
                                   name +
                                   ' could not be established') from exc

        self.__toggle_tracking_service = exc_wrap_call( 
                self.__toggle_tracking_service_id, SetBool)
        self.__get_velocity_scaling_service = exc_wrap_call(
                self.__get_velocity_scaling_service_id, GetFloat)
        self.__set_velocity_scaling_service = exc_wrap_call(
                self.__set_velocity_scaling_service_id, SetFloat)
        self.__get_move_group_name_service = exc_wrap_call(
                self.__get_move_group_name_service_id, GetSelected)
        self.__set_move_group_name_service = exc_wrap_call(
                self.__set_move_group_name_service_id, SetString)
        self.__get_endeffector_name_service = exc_wrap_call(
                self.__get_endeffector_name_service_id, GetSelected)
        self.__set_endeffector_name_service = exc_wrap_call(
                self.__set_endeffector_name_service_id, SetString)
        self.__status_service = exc_wrap_call(
                self.__status_service_id, StatusController)
        self.__get_flag_service = exc_wrap_call(self.__get_flag_service_id, GetFlag)
        self.__set_flag_service = exc_wrap_call(self.__set_flag_service_id, SetFlag)

    def send_set_point(self, setPoint: Pose):
        """
        Jogging to Pose

        Parameters
        ----------
        setPoint : Pose 
            The Pose to which the end effector should jog to
        """

        pose_msg  = setPoint.to_posestamped_msg() 
        self._set_point_pub.publish(pose_msg)

    def send_velocities(self, velocities: JointValues):
        """
        Jogging by applying joint velocities

        Parameters
        ----------
        velocities : JointValues 
            The joint velocities as an instance of JointValues
        """

        point = JointTrajectoryPoint(
            time_from_start = rospy.Duration.from_sec(0.008), 
            velocities = velocities.values)
        trajectory = JointTrajectory(
            joint_names = velocities.joint_set.names,
            points= [point])
        self._jogging_command_pub.publish(trajectory)

    def send_twist(self, twist: Twist):
        """
        Jogging by applying a twist

        Parameters
        ----------
        twist : Twist 
            The twist object which describes the twist 
        """

        twist_stamped = twist.to_twiststamped_msg()
        self._jogging_twist_pub.publish(twist_stamped)


    @staticmethod
    def _exc_wrap_service_call(service_call_func, query_desc, *argv):
        """
        Utility function for a exception handling when calling a service
        """
        try:
            response = service_call_func(*argv)
        except rospy.ServiceException as exc:
            raise ServiceException('service call for query ' +
                                   query_desc +
                                   ' failed, abort') from exc
        return response

    def get_velocity_scaling(self) -> float:
        """
        Get velocity scaling

        Returns
        -------
        float
            The current velocity scaling 

        Raises
        ------
        xamla_motion_exceptions.ServiceException
            If Service not exist or is not callable
        """

        response = self._exc_wrap_service_call(self.__get_velocity_scaling_service, ' get velocity scaling ')
        return response.data

    def set_velocity_scaling(self, value: float) -> None:
        """
        Set velocity scaling

        Parameters
        ----------
        value : float 
            The velocity scaling

        Raises
        ------
        xamla_motion_exceptions.ServiceException
            If Service not exist or is not callable
        """

        response = self._exc_wrap_service_call(self.__set_velocity_scaling_service, 
                                            ' set velocity scaling ',  value)

    def get_move_group_name(self) -> List[str]:
        """
        TODO: verify this, since it is not what one would expected but implemented like this in the csharp jogging client
        Get a list of the names of all move groups 

        Returns
        -------
        List[str]
            List of the names of all move groups 

        Raises
        ------
        xamla_motion_exceptions.ServiceException
            If Service not exist or is not callable
        """

        response = self._exc_wrap_service_call(self.__get_move_group_name_service, 
                                            ' get move group ')
        response.selected
        return response.collection



    def set_move_group_name(self, name: str) -> None:
        """
        Set the name of the current move group used for jogging

        Parameters
        ----------
        name : str 
            The name of the MoveGroup to be set

        Raises
        ------
        xamla_motion_exceptions.ServiceException
            If Service not exist or is not callable
        """

        response = self._exc_wrap_service_call(self.__set_move_group_name_service, 
                                        ' set move group with name {} '.format(name), name)

    def get_endeffector_name(self) -> str:
        """
        TODO: verify this, since it is not what one would expected but implemented like this in the csharp jogging client
        Get a list of the names of all endeffectors

        Returns
        -------
        List[str]
            List of the names of all endeffectors

        Raises
        ------
        xamla_motion_exceptions.ServiceException
            If Service not exist or is not callable
        """

        response = self._exc_wrap_service_call(self.__get_endeffector_name_service, 
                                            ' get endeffector ')
        response.selected
        return response.collection

    def set_endeffector_name(self, name: str) -> None:
        """
        Set the name of the current endeffector used for jogging

        Parameters
        ----------
        name : str 
            The name of the Move endeffector to be set

        Raises
        ------
        xamla_motion_exceptions.ServiceException
            If Service not exist or is not callable
        """
        response = self._exc_wrap_service_call(self.__set_endeffector_name_service, 
                                        ' set endeffector with name {} '.format(name), name)

 #  TODO: Implement this function as well as ControllerStatusModel
 #   def get_status(self) -> ControllerStatusModel:
 #       return None

    def get_flag(self, name: "str") -> bool:
        """
        Get flag for given name

        Parameters
        ----------
        name : str 
            The name of the flag

        Returns
        -------
        bool
            The value of the flag

        Raises
        ------
        xamla_motion_exceptions.ServiceException
            If Service not exist or is not callable
        """

        response = self._exc_wrap_service_call(self.__get_flag_service, 
                                    ' get flag with name {} '.format(name), name)
        return response.value

    def set_flag(self, name: str, value: bool) -> None:
        """
        Set flag for given name

        Parameters
        ----------
        name : str 
            The name of the flag
        value : bool 
            The value for the flag to be set

        Raises
        ------
        xamla_motion_exceptions.ServiceException
            If Service not exist or is not callable
        """

        response = self._exc_wrap_service_call(self.__set_flag_service, 
                                    ' set flag with name {} '.format(name), name, value)

    def toggle_tracking(self, toggle: bool) -> None:
        """
        Toggle tracking on or off

        Parameters
        ----------
        toggle : bool 
            Defines if tracking should be toggled on or off

        Raises
        ------
        xamla_motion_exceptions.ServiceException
            If Service not exist or is not callable
        """

        response = self._exc_wrap_service_call(self.__toggle_tracking_service, 
                                    ' toggle tracking ', toggle)

    def start(self):
        """
        Toggle tracking on

        Raises
        ------
        xamla_motion_exceptions.ServiceException
            If Service not exist or is not callable
        """

        self.toggle_tracking(True)

    def stop(self):
        """
        Toggle tracking off

        Raises
        ------
        xamla_motion_exceptions.ServiceException
            If Service not exist or is not callable
        """

        self.toggle_tracking(False)

    def _handle_jogging_feedback(self, state: ControllerState):
        jogging_state = JoggingClientFeedbackState(
            joint_distance = state.joint_distance,
            cartesian_distance = state.cartesian_distance,
            error_code =  JoggingErrorCode(int(state.error_code)),
            converged = state.converged,
            self_collision_check_enabled = state.self_collision_check_enabled,
            joint_limits_check_enabled = state.joint_limits_check_enabled,
            scene_collision_check_enabled = state.scene_collision_check_enabled)
        self._dispatch(jogging_state)

