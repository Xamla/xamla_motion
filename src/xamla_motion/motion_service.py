# motion_service.py
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
import numpy as np
from datetime import timedelta
import enum

from xamlamoveit_msgs.srv import *
from xamlamoveit_msgs.msg import *
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from moveit_msgs.msg import MoveItErrorCodes
from std_srvs.srv import SetBool

from .xamla_motion_exceptions import ServiceException, ArgumentError
from .data_types import *
from .utility import ROSNodeSteward, LeaseBaseLock
from collections import Iterable

from actionlib_msgs.msg import GoalID

from functools import partial
from threading import Lock
from copy import deepcopy
import asyncio


@enum.unique
class ActionLibGoalStatus(enum.Enum):
    PENDING = 0
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED = 3
    ABORTED = 4
    REJECTED = 5
    PREEMPTING = 6
    RECALLING = 7
    RECALLED = 8
    LOST = 9


@enum.unique
class MotionActionResult(enum.Enum):
    SUCCESS = 1
    FAILURE = 99999
    SIGNAL_LOST = -9999
    PLANNING_FAILED = -1
    INVALID_MOTION_PLAN = -2
    MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE = -3
    CONTROL_FAILED = -4
    UNABLE_TO_AQUIRE_SENSOR_DATA = -5
    TIMED_OUT = -6
    PREEMPTED = -7
    START_STATE_IN_COLLISION = -10
    START_STATE_VIOLATES_PATH_CONSTRAINTS = -11
    GOAL_IN_COLLISION = -12
    GOAL_VIOLATES_PATH_CONSTRAINTS = -13
    GOAL_CONSTRAINTS_VIOLATED = -14
    INVALID_GROUP_NAME = -15
    INVALID_GOAL_CONSTRAINTS = -16
    INVALID_ROBOT_STATE = -17
    INVALID_LINK_NAME = -18
    INVALID_OBJECT_NAME = -19
    FRAME_TRANSFORM_FAILURE = -21
    COLLISION_CHECKING_UNAVAILABLE = -22
    ROBOT_STATE_STALE = -23
    SENSOR_INFO_STALE = -24
    NO_IK_SOLUTION = -31


class SteppedMotionClient(object):

    """
    Client to perform supervised trajectory execution

    Methods
    -------
    moveJ_supervised
        Run supervised trajectory execution
    """

    __next_topic = '/xamlaMoveActions/next'
    __previous_topic = '/xamlaMoveActions/prev'
    __feedback_topic = '/xamlaMoveActions/feedback'
    __movej_action_name = 'moveJ_step_action'

    def __init__(self, trajectory: JointTrajectory,
                 velocity_scaling: float, check_collision: bool=True):
        """
        Run supervised trajectory execution

        Parameters
        ----------
        trajectory : JointTrajectory
            Trajectory which should be executed supervised
        velocity_scaling : float
            Scaling factor to reduce or increase the trajectory 
            velocities range [0.0-1.0]
        check_collision : bool (default True)
            If True collision check is performed 

        Raises
        ------
        TypeError
            If trajectory is not of expected type JointTrajectory
        ValueError
            If velocity_scaling is not in range between 0.0 an 1.0
        ServiceException 
            If action goal handle is not available
        """
        self.__mutex = Lock()

        self.__ros_node_steward = ROSNodeSteward()

        self.__m_action = actionlib.SimpleActionClient(self.__movej_action_name,
                                                       StepwiseMoveJAction)

        if not self.__m_action.wait_for_server(rospy.Duration(5)):
            raise ServiceException('connection to stepped motion action'
                                   ' server could not be established')

        self.__goal_id = None
        self.__progress = None
        self.__state = None

        if not isinstance(trajectory, JointTrajectory):
            raise TypeError('trajectory is not of expected type'
                            ' JointTrajectory')

        goal = StepwiseMoveJGoal()
        velocity_scaling = float(velocity_scaling)
        if velocity_scaling > 1.0 or velocity_scaling < 0.0:
            raise ValueError('velocity scaling is not between 0.0 and 1.0')
        goal.veloctiy_scaling = velocity_scaling
        goal.check_collision = bool(check_collision)
        goal.trajectory = trajectory.to_joint_trajectory_msg()

        loop = asyncio.get_event_loop()
        self.__action_done = loop.create_future()

        def done_callback(goal_status, result):
            status = ActionLibGoalStatus(goal_status)
            if status != ActionLibGoalStatus.SUCCEEDED:
                print('action end unsuccessfully with'
                      ' state: {}'.format(status))
            loop.call_soon_threadsafe(self.__action_done.set_result, result)

        self.__m_action.send_goal(goal, done_cb=done_callback)

        self.__goal_id = self.__m_action.gh.comm_state_machine.action_goal.goal_id

        if not self.__goal_id:
            self.__m_action.cancel_goal()
            raise ServiceException('action goal handle is not available')

        self.__next_pub = rospy.Publisher(self.__next_topic,
                                          GoalID,
                                          queue_size=1)
        self.__previous_pub = rospy.Publisher(self.__previous_topic,
                                              GoalID,
                                              queue_size=1)

        self.__feedback_sub = rospy.Subscriber(self.__feedback_topic,
                                               TrajectoryProgress,
                                               callback=self._feedback_callback,
                                               queue_size=10)

    def __del__(self):
        self.__m_action.cancel_goal()

    @property
    def state(self):
        """
        state : SteppedMotionState or None
            Current state of the supervised trajectory execution
        """
        with self.__mutex:
            return deepcopy(self.__state)

    @property
    def goal_id(self):
        """
        goal_id : GoalId or None
            Current ros action goal id
        """
        return self.__goal_id

    @property
    def action_done_future(self):
        """
        action_done_future : asyncio.future
            future which is done when supervised move is done of cancelled
        """

        return self.__action_done

    def next(self):
        """
        Request at supervised executor to perform next step 
        """
        if self.__goal_id:
            self.__next_pub.publish(self.__goal_id)

    def previous(self):
        """
        Request at supervised executor to perform previous step
        """
        if self.__goal_id:
            self.__previous_pub.publish(self.__goal_id)

    def _feedback_callback(self, trajectory_progress):
        if self.__progress != trajectory_progress.progress:
            with self.__mutex:
                self.__progress = trajectory_progress.progress
                self.__state = SteppedMotionState(self.__goal_id.id,
                                                  trajectory_progress.error_msg,
                                                  trajectory_progress.error_code,
                                                  trajectory_progress.progress)


class MotionService(object):

    __movej_action = 'moveJ_action'
    __query_inverse_kinematics_service = "xamlaMoveGroupServices/query_ik2"

    def __init__(self):

        self.__ros_node_steward = ROSNodeSteward()

        try:
            self.__ik_service = rospy.ServiceProxy(
                self.__query_inverse_kinematics_service,
                GetIKSolution2)

        except rospy.ServiceException as exc:
            raise ServiceException('init service for query'
                                   ' inverse kinematics failed,'
                                   ' abort ') from exc

        self.__m_action = actionlib.SimpleActionClient(self.__movej_action,
                                                       moveJAction)

        if not self.__m_action.wait_for_server(rospy.Duration(5)):
            raise ServiceException('connection to moveJ action'
                                   ' server could not be established')

    @classmethod
    def query_available_move_groups(cls):
        """
        Query all currently available move groups

        To query the move groups the ros service with the string
        defined in query_move_group_service is called

        Returns
        -------
        groups : List[MoveGroupDescription]
            Returns a list with instances of MoveGroupDescription.
            For further details please take a look into the documentation
            of MoveGroupDescription

        Raises
        ------
        xamla_motion.ServiceNotAvailableException
            If Service not exist or is not callable
        TypeError
            If the service response joint_names are not
            of an iterable type
        """

        query_move_group_service = ('xamlaMoveGroupServices/'
                                    'query_move_group_interface')

        try:
            service = rospy.ServiceProxy(
                query_move_group_service,
                QueryMoveGroupInterfaces)
            response = service()
        except rospy.ServiceException as exc:
            raise ServiceException('service call for query'
                                   ' available move groups failed,'
                                   ' abort ') from exc

        groups = list()
        for g in response.move_group_interfaces:
            if len(g.joint_names) == 0:
                joint_set = JointSet.empty()
            else:
                joint_set = JointSet(g.joint_names)
            groups.append(MoveGroupDescription(g.name, g.sub_move_group_ids,
                                               joint_set, g.end_effector_names,
                                               g.end_effector_link_names))
        return groups

    @classmethod
    def query_available_end_effectors(cls):
        """
        Query all currently available end effectors

        To query the available end effectors the ros service with the string
        defined in _query_move_group_service is called an only the relevant
        information about the endeffector is filtered out

        Returns
        -------
        end_effectors : Dict[str, EndEffectorDescription]
            Returns a dictionary key is the endeffector name,
            value is an instance of EndEffectorDescription

        Raises
        ------
        xamla_motion.ServiceExeption
            If Service not exist or is not callable
        TypeError
            If the service response joint_names are not
            of an iterable type
        """
        try:
            move_groups = cls.query_available_move_groups()
        except (ServiceException, TypeError) as exc:
            raise exc

        end_effectors = {}

        for group in move_groups:
            for i, end_effector in enumerate(group.end_effector_names):
                if end_effector not in end_effectors:
                    d = EndEffectorDescription(end_effector,
                                               group.sub_move_group_ids,
                                               group.joint_set, group.name,
                                               group.end_effector_link_names[i])

                    end_effectors[end_effector] = d

        return end_effectors

    @classmethod
    def query_endeffector_limits(cls, name):
        """
        Query end effector limits from ros param

        To query the end effector limits the ros param definied in
        end_effector_limits_param is read out

        Parameters
        ----------
        name : str convertable
            Name of the end effector for which the
            limits are queried

        Returns
        -------
        EndEffectorLimits
            Returns a instance of EndEffectorLimits

        Raises
        ------
        RuntimeError
            If the the ros parameter or
            the requested end effector name
            no exists
        KeyError
            If not all necessary limits exists
        """

        end_effector_limits_param = ('xamlaJointJogging/'
                                     'end_effector_list')

        try:
            eel_param = rospy.get_param(end_effector_limits_param)
        except KeyError as exc:
            raise RuntimeError('end effector limit ros param: '
                               + end_effector_limits_param +
                               ' not exists') from exc
        name = str(name)

        for limits in eel_param:

            try:
                end_effector_name = limits['name']
            except KeyError as exc:
                continue

            if name == end_effector_name:
                max_xyz_vel = limits['taskspace_xyz_max_vel']
                max_xyz_acc = limits['taskspace_xyz_max_acc']
                max_angular_vel = limits['taskspace_angular_max_vel']
                max_angular_acc = limits['taskspace_angular_max_acc']
                return EndEffectorLimits(max_xyz_vel,
                                         max_xyz_acc,
                                         max_angular_vel,
                                         max_angular_acc)
            else:
                continue

        raise RuntimeError('Requested end effector name not exists')

    @classmethod
    def query_joint_limits(cls, joint_set):
        """
        Query end joint limits from ros param

        To query the joint limits the ros param definied in
        joint_limits_param + joint name + limit name is read out

        Parameters
        ----------
        join_set : JointSet
            Set of joints for which the
            limits are queried

        Returns
        -------
        JointLimits
            Returns a instance of JointLimits
            invalid limits are of type numpy.nan

        Raises
        ------
        TypeError : type mismatch
            If joint_set is not of expected type JointSet
        KeyError
            If ros params not exists
        """
        joint_limits_param = ('robot_description_planning/'
                              'joint_limits')

        if not isinstance(joint_set, JointSet):
            raise TypeError('joint_set is not of expected type JointSet')

        maxVel = [None] * len(joint_set)
        maxAcc = [None] * len(joint_set)
        minPos = [None] * len(joint_set)
        maxPos = [None] * len(joint_set)

        for i, name in enumerate(joint_set):
            prefix = joint_limits_param + '/' + name + '/'

            if rospy.get_param(prefix+'has_velocity_limits'):
                maxVel[i] = rospy.get_param(prefix+'max_velocity')

            if rospy.get_param(prefix+'has_acceleration_limits'):
                maxAcc[i] = rospy.get_param(prefix+'max_acceleration')

            if rospy.get_param(prefix+'has_position_limits'):
                minPos[i] = rospy.get_param(prefix+'min_position')
                maxPos[i] = rospy.get_param(prefix+'max_position')

        return JointLimits(joint_set, maxVel, maxAcc, minPos, maxPos)

    @classmethod
    def query_joint_states(cls, joint_set):
        """
        Query joint states by calling the providing ros service

        To query the joint states the ros service with the name
        defined in quary_joint_states_service is called

        Parameters
        ----------
        joint_set : JointSet
            Set of joints for which the joint states are queried

        Returns
        -------
        JointStates
            Returns a instance of JointStates which contains the joint
            states of all joints defined in joint_set

        Raises
        ------
        TypeError : type mismatch
            If joint_set is not of expected type JointSet
        xamla_motion.ServiceException
            If ros service not exists or is not callable
        """
        query_joint_states_service = ('xamlaMoveGroupServices/'
                                      'query_move_group'
                                      '_current_position')

        if not isinstance(joint_set, JointSet):
            raise TypeError('joint_set is not of expected type JointSet')

        try:
            service = rospy.ServiceProxy(
                query_joint_states_service,
                GetCurrentJointState)
            response = service(joint_set.names).current_joint_position
        except rospy.ServiceException as exc:
            print('service call for query current'
                  'joint states failed, abort ')
            raise ServiceException('service call for query'
                                   ' current joint states '
                                   'failed, abort') from exc

        r_joint_set = JointSet(response.name)
        positions = JointValues(r_joint_set,
                                response.position)

        if not response.velocity:
            velocities = None
        else:
            velocities = JointValues(r_joint_set,
                                     response.velocity)

        if not response.effort:
            efforts = None
        else:
            efforts = JointValues(r_joint_set,
                                  response.effort)

        return JointStates(positions, velocities, efforts)

    @classmethod
    def query_pose(cls, move_group_name,
                   joint_positions, end_effector_link=''):
        """
        Computes the pose by applying forward kinematics

        Parameters
        ----------
        move_group_name : str convertable
            name of the move group from which the pose
            is queried
        joint_positions : JointValues
            joint values from which the pose is calculated
        end_effector_link : str convertable (optional)
            end effector link is necessary if end effector
            is not part of the move group but pose should
            be computed for the end effector

        Returns
        -------
        Pose
            Pose of the kinematic chain defined by the input parameter

        Raises
        ------
        TypeError
            If joint_positions is not of type JointValues
        """

        if not isinstance(joint_positions, JointValues):
            raise TypeError('joint_positions is not of'
                            ' expected type JointValues')

        joint_path = JointPath.from_one_point(joint_positions)
        return cls.query_pose_many(move_group_name, joint_path,
                                   end_effector_link)[0]

    @classmethod
    def query_pose_many(cls, move_group_name,
                        joint_path, end_effector_link=''):
        """
        Query the poses from joint path points by applying forward kinematics

        Parameters
        ----------
        move_group_name : str convertable
            name of the move group from which the poses are queried
        joint_path : JointPath
            joint path from which the poses are calculated
        end_effector_link : str convertable (optional)
            end effector link is necessary if end effector
            is not part of the move group but pose should
            be computed for the end effector

        Returns
        -------
        List[Pose]
            List of Poses

        Raises
        ------
        TypeError
            If joint_path is not of type JointPath
        """

        query_forward_kinematics_service = ('xamlaMoveGroupServices/'
                                            'query_fk')

        move_group_name = str(move_group_name)
        end_effector_link = str(end_effector_link)

        if not isinstance(joint_path, JointPath):
            raise TypeError('joint_path is not of expected type JointPath')

        try:
            service = rospy.ServiceProxy(
                query_forward_kinematics_service,
                GetFKSolution)
            response = service(move_group_name,
                               end_effector_link,
                               joint_path.joint_set,
                               [p.to_joint_path_point_msg()
                                for p in joint_path])
        except rospy.ServiceException as exc:
            print('service call for query forward kinematics'
                  ' failed, abort ')
            raise ServiceException('service call for query'
                                   ' forward kinematics'
                                   ' failed, abort') from exc

        if (response.error_codes == None or response.error_msgs == None
                or len(response.error_codes) != len(response.error_msgs)):
            raise ServiceException('service call for query forward'
                                   'kinematics returns with '
                                   'invalid response')

        for i, error in enumerate(response.error_codes):
            if error.val != MoveItErrorCodes.SUCCESS:
                raise ServiceException('service call for query forward'
                                       ' kinematics was not'
                                       ' successful for point: ' + str(i) +
                                       'service name:' +
                                       query_forward_kinematics_service +
                                       ' error code: ' +
                                       str(response.error_code.val))

        return list(map(lambda x: Pose.from_posestamped_msg(x),
                        response.solutions))

    @classmethod
    def _query_moveit_joint_path(cls, move_group_name, joint_path):

        query_joint_path_service = ('xamlaPlanningServices/'
                                    'query_joint_path')

        if not isinstance(joint_path, JointPath):
            raise TypeError('joint_path is not of expected type JointPath')

        try:
            service = rospy.ServiceProxy(
                query_joint_path_service,
                GetMoveItJointPath)
            response = service(move_group_name,
                               joint_path.joint_set,
                               [p.to_joint_path_point_msg()
                                for p in joint_path])
        except rospy.ServiceException as exc:
            print('service call for query joint path'
                  ' failed, abort ')
            raise ServiceException('service call for query'
                                   ' joint path'
                                   ' failed, abort') from exc

        return response

    @classmethod
    def query_collision_free_joint_path(cls, move_group_name, joint_path):
        """
        Query a collision free joint path from a user defined joint path

        Parameters
        ----------
        move_group_name : str convertable
            name of the move group for which the collision free joint
            is required
        joint_path : JointPath
            joint path which may contains collisions

        Results
        -------
        JointPath
            New planned JointPath without collisions

        Raises
        ------
        TypeError : type mismatch
            If joint_path is not of expectef type JointPath
            or it move_group_name is not convertable to str

        ServiceException
            If query joint path service is not callable or
            is not successful
        """

        query_joint_path_service = ('xamlaPlanningServices/'
                                    'query_joint_path')

        if not isinstance(joint_path, JointPath):
            raise TypeError('joint_path is not of expected type JointPath')

        move_group_name = str(move_group_name)

        response = cls._query_moveit_joint_path(move_group_name,
                                                joint_path)

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise ServiceException('service call for query collision free'
                                   ' joint path was not successful. '
                                   'service name:' +
                                   query_joint_path_service +
                                   ' error code: ' +
                                   str(response.error_code.val))

        return JointPath(joint_path.joint_set,
                         [JointValues(joint_path.joint_set, p.positions)
                          for p in response.path])

    @classmethod
    def query_joint_trajectory(cls, joint_path, max_velocity, max_acceleration,
                               max_deviation, delta_t):
        """
        Query a joint trajectory from joint path / joint positions

        Parameters
        ----------
        joint_path : JointPath
            Defines the key joint positions the trajectory must reach
        max_velocity : Iterable[float convertable]
            Defines the maximal velocity for every joint
        max_acceleration : Iterable[float convertable]
            Defines the maximal acceleration for every joint
        max_deviation : float convertable
            Defines the maximal deviation of the joints to the
            defined key points while executing the trajectory
        delta_t : float convertable
            Sampling points frequency or time
            if value is create as 1.0 the value is interpreted
            as a value in seconds else the value is interpreted
            as a value in Hz

        Returns
        -------
        JointTrajectory
            An instance of JointTrajectory with dense JointTrajectoryPoints

        Raises
        ------
        TypeError
            If joint_path is not of expected type JointPath or
            max_velocity or max_acceleration is not iterable or
            values of max_velocity, max_acceleration or delta_t
            are not convertable to float
        ValueError
            If max_velocity or max_acceleration have not the same
            number of values has joints are defined in joint_path
        """

        query_joint_trajectory_service = ('xamlaPlanningServices/'
                                          'query_joint_trajectory')

        if not isinstance(joint_path, JointPath):
            raise TypeError('joint_path is not of expected type JointPath')

        max_velocity = [float(v) for v in max_velocity]
        if len(max_velocity) != len(joint_path.joint_set):
            raise ValueError('max_velocity has not the same number of values'
                             ' as joints are defined in joint_path')

        max_acceleration = [float(v) for v in max_acceleration]
        if len(max_acceleration) != len(joint_path.joint_set):
            raise ValueError('max_acceleration has not the same number of'
                             ' values as joints are defined in joint_path')

        max_deviation = float(max_deviation)
        delta_t = float(delta_t)

        delta_t = 1 / delta_t if delta_t > 1.0 else delta_t

        try:
            service = rospy.ServiceProxy(
                query_joint_trajectory_service,
                GetOptimJointTrajectory)
            response = service(joint_path.joint_set,
                               [p.to_joint_path_point_msg()
                                for p in joint_path],
                               max_velocity,
                               max_acceleration,
                               max_deviation,
                               delta_t)
        except rospy.ServiceException as exc:
            print('service call for query joint trajectory'
                  ' failed, abort ')
            raise ServiceException('service call for query'
                                   ' joint trajectory'
                                   ' failed, abort') from exc

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise ServiceException('service call for query joint'
                                   ' trajectory was not successful. '
                                   'service name:' +
                                   query_joint_trajectory_service +
                                   ' error code: ' +
                                   str(response.error_code.val))

        j = JointSet(response.solution.joint_names)

        p = [JointTrajectoryPoint.from_joint_trajectory_point_msg(j, p)
             for p in response.solution.points]

        return JointTrajectory(j, p)

    @classmethod
    def query_task_space_trajectory(cls, end_effector_name, cartesian_path, seed,
                                    max_xyz_velocity, max_xyz_acceleration,
                                    max_angular_velocity, max_angular_acceleration,
                                    ik_jump_threshold, max_deviation, collision_check,
                                    delta_t):
        """
        Query a joint trajectory from task space poses

        Parameters
        ----------
        end_effector_name : str convertable
            Name of the end effector for a trajectory should be quried
        cartesian_path : CartesianPath
            Define the key poses the trajectory must reach
        seed : JointValues
            numerical seed to control configuration of the robot
        max_xyz_velocity : float convertable
            max velocity for translation in m/s
        max_xyz_acceleration : float convertable
            max acceleration for translation in m/s^2
        max_angular_velocity : float convertable
            max angular velocity in rad/s
        max_angular_acceleration : float convertable
            max angular acceleration in rad/s^2
        ik_jump_threshold : float convertable
            maximal inverse kinematic jump
        max_deviation : float convertable
            Defines the maximal deviation of the joints to the
            defined key points while executing the trajectory
        collision_check : bool convertable
            If true check the trajectory is collision free
        delta_t : float convertable
             Sampling points frequency or time
            if value is create as 1.0 the value is interpreted
            as a value in seconds else the value is interpreted
            as a value in Hz

        Returns
        -------
        JointTrajectory
            An instance of JointTrajectory with dense JointTrajectoryPoints

        Raises
        ------
        TypeError
            If cartesian_path is not of expected type CartesianPath or
            if seed is not of expected type JointValues
            max_* is not iterable or the values are not convertable
            as expected
        ValueError
            If max_velocity or max_acceleration have not the same
            number of values has joints are defined in joint_path
        """

        query_cartesian_trajectory_service = ('xamlaPlanningServices/'
                                              'query_cartesian_trajectory')

        if not isinstance(cartesian_path, CartesianPath):
            raise TypeError(
                'cartesian_path is not of expected type CartesianPath')

        if not isinstance(seed, JointValues):
            raise TypeError('seed is not of expected type JointValues')

        max_xyz_velocity = float(max_xyz_velocity)

        max_xyz_acceleration = float(max_xyz_acceleration)

        max_angular_velocity = float(max_angular_velocity)

        max_angular_acceleration = float(max_angular_acceleration)

        end_effector_name = str(end_effector_name)
        ik_jump_threshold = float(ik_jump_threshold)
        max_deviation = float(max_deviation)
        collision_check = bool(collision_check)
        delta_t = float(delta_t)

        delta_t = 1 / delta_t if delta_t > 1.0 else delta_t

        try:
            service = rospy.ServiceProxy(
                query_cartesian_trajectory_service,
                GetLinearCartesianTrajectory)
            response = service(end_effector_name,
                               [p.to_posestamped_msg()
                                for p in cartesian_path],
                               max_xyz_velocity,
                               max_xyz_acceleration,
                               max_angular_velocity,
                               max_angular_acceleration,
                               delta_t,
                               ik_jump_threshold,
                               max_deviation,
                               seed.joint_set,
                               seed.to_joint_path_point_msg(),
                               collision_check)
        except rospy.ServiceException as exc:
            print('service call for query cartesian trajectory'
                  ' failed, abort ')
            raise ServiceException('service call for query'
                                   ' cartesian trajectory'
                                   ' failed, abort') from exc

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise ServiceException('service call for query cartesian'
                                   ' trajectory was not successful. '
                                   'service name:' +
                                   query_cartesian_trajectory_service +
                                   ' error code: ' +
                                   str(response.error_code.val))

        j = JointSet(response.solution.joint_names)

        p = [JointTrajectoryPoint.from_joint_trajectory_point_msg(j, p)
             for p in response.solution.points]

        return JointTrajectory(j, p)

    @classmethod
    def query_joint_path_collisions(cls, move_group_name, joint_path):
        """
        Query collisions in joint path

        Parameters
        ----------
        move_group_name : str convertable
            Name of the move group for which a path should be check
            for collisions
        joint_path : JointPath
            The path that should be check for collisions

        Returns
        -------
        List[JointValuesCollision] or None
            If collisions exists returns a list of
            JointValuesCollisions else returns None

        Raises
        ------
        TypeError
            If move_group_name ist not str convertable or
            if joint_path is not of type JointPath
        """

        query_joint_path_collisions = ('xamlaMoveGroupServices/'
                                       'query_joint_position_collision_check')

        move_group_name = str(move_group_name)

        if not isinstance(joint_path, JointPath):
            raise TypeError('joint_path is not of expected type JointPath')

        try:
            service = rospy.ServiceProxy(
                query_joint_path_collisions,
                QueryJointStateCollisions)
            response = service(move_group_name,
                               joint_path.joint_set,
                               [p.to_joint_path_point_msg()
                                for p in joint_path])
        except rospy.ServiceException as exc:
            print('service call for query joint collisions'
                  ' failed, abort ')
            raise ServiceException('service call for query'
                                   ' joint collisions'
                                   ' failed, abort') from exc

        response.in_collision = list(response.in_collision)
        response.error_codes = list(response.error_codes)
        if (len(response.in_collision) != len(joint_path) or
            len(response.error_codes) != len(joint_path) or
                len(response.messages) != len(joint_path)):
            raise ServiceException('service call for query joint'
                                   ' collisions was not successful. '
                                   'service name:' +
                                   query_joint_path_collisions +
                                   ' error code: ' +
                                   str(response.error_code.val))

        result = [JointValuesCollisions(i, response.error_codes[i],
                                        response.messages[i])
                  for i in range(0, len(joint_path))
                  if response.in_collision[i]]

        return result

    @classmethod
    def create_plan_parameters(cls, move_group_name=None, joint_set=None,
                               max_velocity=None, max_acceleration=None,
                               **kwargs):
        """
        Create PlanParameters from user defined and/or quried inputs

        Parameters
        ----------
        move_group_name : str (optional  default query first)
            move group for which plan parameters should be created
        joint_set : JointSet (optional default query similar)
            joint set for which plan parameters should be created
        max_velocity : Iterable[float convertable] (optional default max query)
            Defines the maximal velocity for every joint
        max_acceleration : Iterable[float convertable] (op default max query)
            Defines the maximal acceleration for every joint
        kwargs : dict
            sample_resolution : float convertable (optional default 0.008/125Hz)
                sample points frequency
                if value is create as 1.0 the value is interpreted
                as a value in seconds else the value is interpreted
                as a value in Hz
            collision_check : bool convertable (optional default True)
                check for collision if True
            max_deviation : float convertable (optional default 0.2)
                max deviation from fly by points
            velocity_scaling : float convertable (optional default 1.0)
                scale query or user defined max velocity
                values between 0.0 and 1.0
            acceleration_scaling : float convertable (optional default 1.0)
                scale query or user defined max acceleration
                values between 0.0 and 1.0

        Returns
        -------
        PlanParameters
            Instance of plan parameters with automatically
            queried and/or user defined values

        Raises
        ------
        ServiceError
            If query services are not reachable or
            not finish successful
        TypeError
            If move group name is not of type None or str convertable
            If joint_set is not of type None or JointSet
            If an other parameter is not convertable
        ValueError
            If scaling parameters are not between 0.0 and 1.0
        ArgumentError
            If a argument is not set and also could not be
            quired automatically
        """

        if not move_group_name:
            groups = cls.query_available_move_groups()
            if groups:
                joint_set = joint_set if joint_set else groups[0].joint_set
                for g in groups:
                    if g.joint_set.is_similar(joint_set):
                        move_group_name = g.name
                        break
            else:
                raise ArgumentError('no move group name was provided and'
                                    ' also no move group could'
                                    ' automatically be quired')
        elif not joint_set:
            groups = cls.query_available_move_groups()
            for g in groups:
                if g.name == move_group_name:
                    joint_set = g.joint_set
                    break
            if not joint_set:
                raise ArgumentError('no joint_set was provided and'
                                    ' also no joint_set could'
                                    ' automatically be quired')

        if not max_velocity or not max_acceleration:
            limits = cls.query_joint_limits(joint_set)
            if not max_velocity:
                max_velocity = limits.max_velocity
            if not max_acceleration:
                max_acceleration = limits.max_acceleration

        joint_limits = JointLimits(joint_set, max_velocity,
                                   max_acceleration, None, None)

        return PlanParameters(move_group_name, joint_limits, **kwargs)

    @classmethod
    def create_task_space_plan_parameters(cls, end_effector_name=None,
                                          max_xyz_velocity=None,
                                          max_xyz_acceleration=None,
                                          max_angular_velocity=None,
                                          max_angular_acceleration=None,
                                          **kwargs):
        """
        Creates TakesSpacePlanParameters from user defined or queried inputs

        Parameters
        ----------
        move_group_name : str (optional default query first)
            move group for which plan parameters should be created
        max_xyz_velocity : float convertable or None
            Defines the maximal xyz velocity [m/s]
        max_xyz_acceleration : float convertable
            Defines the maximal xyz acceleration [m/s^2]
        max_angular_velocity : float convertable or None
            Defines the maximal angular velocity [rad/s]
        max_angular_acceleration : float convertable or None
            Defines the maximal angular acceleration [rad/s^2]
        kwargs : dict
            sample_resolution : float convertable (optional default 0.008/125Hz)
                sample points frequency
                if value is create as 1.0 the value is interpreted
                as a value in seconds else the value is interpreted
                as a value in Hz
            collision_check : bool convertable (optional default True)
                check for collision if True
            ik_jump_threshold : float convertable (optional default 1.2)
                maximal inverse kinematic jump
            max_deviation : float convertable (optional default 0.2)
                max deviation from fly by points
            velocity_scaling : float convertable (optional default 1.0)
                scale query or user defined max velocity
                values between 0.0 and 1.0
            acceleration_scaling : float convertable (optional default 1.0)
                scale query or user defined max acceleration
                values between 0.0 and 1.0

        Returns
        -------
        TaskSpacePlanParameters
            Instance of TaskSpacePlanParameters with automatically
            queried and/or user defined values

        Raises
        ------
        ServiceError
            If query services are not reachable or
            not finish successful
        TypeError
            If end_effector_name is not of type None or str convertable
            If an other parameter is not convertable
        ValueError
            If scaling parameters are not between 0.0 and 1.0
        ArgumentError
            If a argument is not set and also could not be
            quired automatically
        """

        if not end_effector_name:
            groups = cls.query_available_move_groups()
            if groups:
                for g in groups:
                    if g.end_effector_link_names:
                        end_effector_name = g.end_effector_names[0]
                        break
            if not end_effector_name:
                raise ArgumentError('no end effector name was provided and'
                                    ' also end effector name could'
                                    ' automatically be quired')

        if (not max_xyz_velocity or not max_xyz_acceleration or
                not max_angular_velocity or not max_angular_acceleration):
            limits = cls.query_endeffector_limits(end_effector_name)

            if not max_xyz_velocity:
                max_xyz_velocity = limits.max_xyz_velocity
            if not max_xyz_acceleration:
                max_xyz_acceleration = limits.max_xyz_acceleration
            if not max_angular_velocity:
                max_angular_velocity = limits.max_angular_velocity
            if not max_angular_acceleration:
                max_angular_acceleration = limits.max_angular_acceleration

        end_effector_limits = EndEffectorLimits(max_xyz_velocity,
                                                max_xyz_acceleration,
                                                max_angular_velocity,
                                                max_angular_acceleration)

        return TaskSpacePlanParameters(end_effector_name,
                                       end_effector_limits,
                                       **kwargs)

    @classmethod
    def plan_collision_free_joint_path(cls, path, parameters):
        """
        Plans a collision free joint path by query it

        Parameters
        ----------
        path : JointPath
            joint path which should be replanned to be
            collision free
        parameters : PlanParameters
            plan parameters which defines the limits and
            move group

        Returns
        -------
        JointPath
            the replanned collision free joint path

        Raises
        ------
        TypeError
            If path is not of type JointPath or
            if parameters is not of type PlanParameters
        ValueError
            If parameters joint set is not equal or sub
            set of the path joint set and therefore
            reordering was not possible
        ServiceError
            If query service is not available or finish
            unsuccessfully
        """

        if not isinstance(path, JointPath):
            raise TypeError('path is not of expected'
                            ' type JointPath')

        if not isinstance(parameters, PlanParameters):
            raise TypeError('parameters is not of expected'
                            ' type PlanParameters')

        # reorder path in respect to plan parameter joint_set
        path = JointPath(parameters.joint_set, path.points)

        return cls.query_collision_free_joint_path(parameters.move_group_name,
                                                   path)

    @classmethod
    def plan_move_pose_linear(cls, path, seed, parameters):
        """
        Plans trajectory with linear movements from a cartesian path

        Parameters
        ----------
        path : CartesianPath
            cartesian path with poses the trajectory must reach
        seed : JointValues
            numerical seed to control configuration
        parameters : TaskSpacePlanParameters
            plan parameters which defines the limits, settings
            and end effector name

        Returns
        -------
        JointTrajectory
            Planned joint trajectory which reach the poses
            defined in path under the constraints of
            parameters

        Raises
        ------
        TypeError
            If path is not of type CartesianPath or
            if parameters is not of type TaskSpacePlanParameters or
            if seed is not of type JointValues
        ServiceError
            If query service is not available or finish
            unsuccessfully
        """

        if not isinstance(path, CartesianPath):
            raise TypeError('path is not of expected type CartesianPath')

        if not isinstance(seed, JointValues):
            raise TypeError('seed is not of expected type JointValues')

        if not isinstance(parameters, TaskSpacePlanParameters):
            raise TypeError('parameters is not of expected type'
                            ' TaskSpacePlanParameters')

        p = parameters
        return cls.query_task_space_trajectory(p.end_effector_name,
                                               path,
                                               seed,
                                               p.max_xyz_velocity,
                                               p.max_xyz_acceleration,
                                               p.max_angular_velocity,
                                               p.max_angular_acceleration,
                                               p.ik_jump_threshold,
                                               p.max_deviation,
                                               p.collision_check,
                                               p.sample_resolution)

    @classmethod
    def plan_move_joints(cls, path, parameters):
        """
        Plans trajectory from a joint path

        Parameters
        ----------
        path : JointPath
            joint path with positions the trajectory must reach
        parameters : PlanParameters
            plan parameters which defines the limits, settings
            and move group name

        Returns
        -------
        JointTrajectory
            Planned joint trajectory which reach the positions
            defined in path under the constraints of
            parameters

        Raises
        ------
        TypeError
            If path is not of type JointPath or
            if parameters is not of type PlanParameters or
        ValueError
            If parameters joint set is not equal or sub
            set of the path joint set and therefore
            reordering was not possible
        ServiceError
            If query service is not available or finish
            unsuccessfully
        """

        if not isinstance(path, JointPath):
            raise TypeError('path is not of expected type JointPath')

        if not isinstance(parameters, PlanParameters):
            raise TypeError('parameters is not of expected type'
                            ' PlanParameters')

        path = JointPath(parameters.joint_set, path.points)

        max_velocity = parameters.max_velocity
        max_acc = parameters.max_acceleration

        vel_isnan = np.isnan(parameters.max_velocity)
        acc_isnan = np.isnan(parameters.max_acceleration)
        if any(vel_isnan) or any(acc_isnan):
            limits = cls.query_joint_limits(path.joint_set)

            if any(vel_isnan):
                max_velocity = [limits.max_velocity[i] *
                                parameters.velocity_scaling
                                if vel_isnan[i] else l
                                for i, l in enumerate(parameters.max_velocity)]

            if any(acc_isnan):
                max_acc = [limits.max_acceleration[i] *
                           parameters.acceleration_scaling
                           if acc_isnan[i] else l
                           for i, l in enumerate(parameters.max_acceleration)]

        return cls.query_joint_trajectory(path,
                                          max_velocity,
                                          max_acc,
                                          parameters.max_deviation,
                                          parameters.sample_resolution)

    async def execute_joint_trajectory(self, trajectory, collision_check):
        """
        Executes a joint trajectory

        Parameters
        ----------
        trajectory : JointTrajectory 
            Joint trajectory which should be executed
        collision_check : bool convertable
            If True check for collision while executing

        Returns
        -------
        result : int
            Result of the trajectory execution

        Raises
        ------
        TypeError 
            If trajectory is not of type JointTrajectory
            or if collision_check is not convertable to bool
        """

        if not isinstance(trajectory, JointTrajectory):
            raise TypeError('trajectory is not of expected type'
                            ' joint trajectory')

        collision_check = bool(collision_check)

        goal = moveJGoal(trajectory=trajectory.to_joint_trajectory_msg(),
                         check_collision=collision_check)

        run_action = self._generate_action_executor(self.__m_action)

        # the server itself lock resources
        # with LeaseBaseLock(trajectory.joint_set.names) as lock_resources:
        response = await run_action(goal)

        if not response:
            raise RuntimeError('Unexpected result received by'
                               ' SimpleActionClient for moveJ action')

        return response.result()

    def execute_joint_trajectory_supervised(self, trajectory: JointTrajectory,
                                            velocity_scaling: float,
                                            collision_check: bool) -> SteppedMotionClient:
        """
        Creates a instance of SteppedMotionClient for supervised trajectory execution
        Parameters
        ----------
        trajectory : JointTrajectory 
            Joint trajectory which should be executed
        velocity_scaling : float
            scaling of velocity range 0.0 to 1.0
        collision_check : bool convertable
            If True check for collision while executing

        Returns
        -------
        result : int
            Result of the trajectory execution

        Raises
        ------
        TypeError 
            If trajectory is not of type JointTrajectory
            or if collision_check is not convertable to bool
        """

        return SteppedMotionClient(trajectory, velocity_scaling, collision_check)

    def query_inverse_kinematics(self, pose, parameters,
                                 seed=[],
                                 end_effector_link='',
                                 timeout=None,
                                 attempts=1):
        """
        Query inverse kinematic solutions one pose

        Parameters
        ----------
        pose : pose
            Pose to transform to joint space
        parameters : PlanParameters
            Plan parameters which defines the limits, settings
            and move group name
        seed : JointValues (optional)
            Numerical seed to control joint configuration
        end_effector_link : str convertable (optinal)
            necessary if poses are defined for end effector link
        timeout : datatime.timedelta
            timeout
        attempts : int convertable
            Attempts to find a solution or each pose

        Returns
        -------
        JointValues
            Instance of JointValues which is the joint space
            equivalent of the task space pose  

        Raises
        ------
        TypeError
            If pose is not of type Pose or
            parameters is not of type PlanParameters
            or seed is not empty list or type JointValues
            or the other parameters are not convertable
            to defined types
        ValueError
            If parameters joint set is not equal or sub
            set of the seed joint set if defined and therefore
            reordering was not possible
        ServiceError
            If query service is not available
        """

        if not isinstance(pose, Pose):
            raise TypeError('pose is not of expected type Pose')

        ipath = CartesianPath.from_one_point(pose)

        result = self.query_inverse_kinematics_many(ipath,
                                                    parameters,
                                                    seed,
                                                    end_effector_link,
                                                    timeout,
                                                    attempts)

        if not result.succeeded:
            raise ServiceException('ik service call failed with error'
                                   ' code: {}'.format(result.error_codes[0]))

        return result.path[0]

    def query_inverse_kinematics_many(self, path, parameters,
                                      seed=[],
                                      end_effector_link='',
                                      timeout=None,
                                      attempts=1,
                                      const_seed=False):
        """
        Query inverse kinematic solutions for all point in path

        Parameters
        ----------
        path : CartesianPath
            Path with poses to transform to joint space
        parameters : PlanParameters
            Plan parameters which defines the limits, settings
            and move group name
        seed : JointValues (optional)
            Numerical seed to control joint configuration
        end_effector_link : str convertable (optinal)
            necessary if poses are defined for end effector link
        timeout : datatime.timedelta
            timeout
        attempts : int convertable
            Attempts to find a solution or each pose
        const_seed : bool convertable
            todo

        Returns
        -------
        IkResult
            Instance of IkResult with all found solutions as
            a JointPath and error codes

        Raises
        ------
        TypeError
            If path is not of type CartesianPath or
            parameters is not of type PlanParameters
            or seed is not empty list or type JointValues
            or the other parameters are not convertable
            to defined types
        ValueError
            If parameters joint set is not equal or sub
            set of the seed joint set if defined and therefore
            reordering was not possible
        ServiceError
            If query service is not available
        """

        if not isinstance(path, CartesianPath):
            raise TypeError('path is not of expected type CartesianPath')

        if not isinstance(parameters, PlanParameters):
            raise TypeError('parameters is not of expected'
                            ' type PlanParameters')

        if seed:
            if not isinstance(seed, JointValues):
                raise TypeError('seed is not of expected'
                                ' type JointValues')
            elif seed.joint_set == parameters.joint_set:
                pass
            elif parameters.joint_set.is_subset(seed.joint_set):
                seed = seed.reorder(parameters.joint_set)
            else:
                raise ValueError('joint set of parameters and seed do not'
                                 ' match and reording is not possible')

        if timeout and not isinstance(timeout, timedelta):
            raise TypeError('timeout is not of expected type timedelta')
        else:
            timeout = timedelta(milliseconds=200)

        end_effector_link = str(end_effector_link)
        attempts = int(attempts)
        const_seed = bool(const_seed)

        duration = self._ros_duration_from_timedelta(timeout)

        poses_msgs = []

        for p in path:
            poses = EndEffectorPoses()
            poses.poses.append(p.to_posestamped_msg())
            poses.link_names.append(end_effector_link)
            poses_msgs.append(poses)

        req = GetIKSolution2Request()
        req.group_name = parameters.move_group_name
        req.joint_names = parameters.joint_set.names
        req.seed = seed.to_joint_values_point_msg()
        req.const_seed = const_seed
        req.points = poses_msgs
        req.collision_check = parameters.collision_check
        req.attemts = attempts
        req.timeout = duration

        try:
            response = self.__ik_service(req)
        except rospy.ServiceException as exc:
            print('service call for query inverse kinematics'
                  ' failed, abort ')
            raise ServiceException('service call for query'
                                   ' inverse kinematics'
                                   ' failed, abort') from exc

        f = JointValues.from_joint_path_point_msg
        joint_path = JointPath(parameters.joint_set,
                               [f(parameters.joint_set, p)
                                for p in response.solutions])

        error_codes = [e.val if e.val != 0 else MoveItErrorCodes.FAILURE
                       for e in response.error_codes]

        return IkResults(joint_path, error_codes)

    @staticmethod
    def _ros_duration_from_timedelta(timedelta):
        secs = timedelta.days*24*3600+timedelta.seconds
        nsecs = timedelta.microseconds*1000
        return rospy.Duration(secs, nsecs)

    def plan_cartesian_path(self, path, parameters):
        """
        Plan a joint trajectory from a cartesian path and plan parameters

        Parameters
        ----------
        path : CartesianPath
            Poses the planned trajectory must reach
        parameters : PlanParameters
            Plan parameters which defines the limits, settings
            and move group name

        Returns
        -------
        JointTrajectory
            A joint trajectory which reach defined poses of path
            under the constrains in parameters


        Raises
        ------
        TypeError
            If path is not of type CartesianPath or
            parameters is not of type PlanParameters
        ValueError
            If parameters joint set is not equal or sub
            set of the seed joint set and therefore
            reordering was not possible
        ServiceError
            If query services are not available or
            finish unsuccessfully or inverse kinematics
            ends with error
        """

        seed = self.query_joint_states(parameters.joint_set).positions
        result = self.query_inverse_kinematics_many(path,
                                                    parameters,
                                                    seed)

        if not result.succeeded:
            raise ServiceException('service call for query inverse'
                                   ' kinematics was not successful')

        return self.plan_collision_free_joint_path(result.path,
                                                   parameters)

    @classmethod
    def emergency_stop(cls, enable=True):
        """
        Sets and resets emergency stop

        Parameters
        ----------
        enable : bool convertable (default True)
            If True activate emergency stop
            If False resets the emergency stop

        Returns
        -------
        success : bool
            True if the service call was successful
        message : str
            Response or error message

        Raises
        ------
        TypeError
            If enable is not convertable to bool
        ServiceError
            If query service is not available
        """

        query_emergency_stop = ('EmergencySTOP/'
                                'query_emergency_stop')

        enable = bool(enable)

        try:
            service = rospy.ServiceProxy(
                query_emergency_stop,
                SetBool)
            response = service(enable)
        except rospy.ServiceException as exc:
            print('service set emergency stop failed')
            raise ServiceException('service call for set emergency'
                                   ' stop failed, abort') from exc

        return response.success, response.message

    @classmethod
    def get_current_joint_values(cls, joint_set):
        """
        Query the current joint positions of all joints in joint_set

        Parameters
        ----------
        joint_set : JointSet
            Defines for which joint the positions are requested

        Returns
        ------
        positions : JointValues
            Returns a instance of JointValues with the positions
            of the requested joint set

        Raises
        ------
        TypeError
            If joint_set is not of type JointSet
        """
        print(rospy.get_name())
        return cls.query_joint_states(joint_set).positions

    # async def move_joints(self, target, parameters):
    #     """
    #     Moves joints to target joint positions

    #     Parameters
    #     ----------
    #     target : JointValues
    #         target joint positions
    #     parameters : PlanParameters
    #         Plan parameters which defines the limits, settings
    #         and move group name

    #     Returns
    #     -------
    #     None

    #     Raises
    #     ------
    #     TypeError
    #         If target is not of type JointValues or
    #         if parameters is not of type PlanParameters
    #     ServiceError
    #         If query services are not available or finish
    #         with a fail state
    #     """

    #     if not isinstance(target, JointValues):
    #         raise TypeError('target is not of expected type JointValues')

    #     if not isinstance(parameters, PlanParameters):
    #         raise TypeError('parameters is not of expected'
    #                         ' type PlanParameters')

    #     source = self.get_current_joint_values(parameters.joint_set)
    #     path = JointPath.from_start_stop_point(source, target)
    #     trajectory = self.plan_move_joints(path, parameters)
    #     await self.execute_joint_trajectory(trajectory, parameters.collision_check)

    # async def move_pose(self, target, end_effector_link, parameters, seed=None):
    #     """
    #     Moves to target pose

    #     Parameters
    #     ----------
    #     target : Pose
    #         target pose
    #     end_effector_link : str convertable
    #         specifies for which link the pose is defined
    #     parameters : PlanParameters
    #         Plan parameters which defines the limits, settings
    #         and move group name
    #     seed : JointValues or None
    #         numerical seed to control the robot configuration
    #         if none the current joint position is used

    #     Raises
    #     ------
    #     TypeError
    #         If target is not of type Pose
    #         If parameters is not of type PlanParameters
    #         If end_effector_link is not convertable to str
    #         If seed is not of type JointValues
    #     ServiceError
    #         If query services are not available or finish
    #         with a fail state
    #     """

    #     if not isinstance(target, Pose):
    #         raise TypeError('target is not of expected type Pose')

    #     if not seed:
    #         seed = self.get_current_joint_values(parameters.joint_set)

    #     joint_values = self.query_inverse_kinematics(target, parameters,
    #                                                  seed,
    #                                                  end_effector_link)

    #     await self.move_joints(joint_values, parameters)

    # async def move_pose_linear(self, target, end_effector_link, parameters, seed=None):
    #     """
    #     Moves to target pose linear

    #     Parameters
    #     ----------
    #     target : Pose
    #         target pose
    #     end_effector_link : str convertable
    #         specifies for which link the pose is defined
    #     parameters : TaskPlanParameters
    #         Task space plan parameters which defines the limits, settings
    #         and end effector name
    #     seed : JointValues
    #         numerical seed to control the robot configuration

    #     Raises
    #     ------
    #     TypeError
    #         If target is not of type Pose
    #         If parameters is not of type TaskSpacePlanParameters
    #         If end_effector_link is not convertable to str
    #         If seed is not of type JointValues or None
    #     ServiceError
    #         If query services are not available or finish
    #         with a fail state
    #     """

    #     groups = self.query_available_move_groups()
    #     group = next(g for g in groups
    #                  if any([parameters.end_effector_name in
    #                          g.end_effector_names]))

    #     if not group:
    #         raise RuntimeError('no move group is available with'
    #                            ' requested end effector name: ' +
    #                            parameters.end_effector_name)

    #     if not seed:
    #         seed = self.get_current_joint_values(group.joint_set)

    #     source = self.query_pose(group.name, seed, end_effector_link)
    #     path = CartesianPath.from_start_stop_point(source, target)
    #     trajectory = self.plan_move_pose_linear(path, seed, parameters)
    #     await self.execute_joint_trajectory(trajectory,
    #                                         parameters.collision_check)

    async def move_gripper(self, action_name, position, max_effort):
        """
        Moves a gripper via the general ros interface GripperCommandAction

        Parameters
        ----------
        action_name : str convertable
            Name of the action to control a specific gripper
        position : float convertable
            Requested position of the gripper in meter
        max_effort : float convertable
            Force which should be applied

        Results
        -------
        action_response : MoveGripperResult
            Result of the action execution

        Raises
        ------
        TypeError
            If inputs are not convertable to specified types
        ServiceError
            If action server is not available
        RuntimeError
            If action returns unexpected result
        """

        action_name = str(action_name)

        action_client = actionlib.SimpleActionClient(action_name,
                                                     GripperCommandAction)

        g = GripperCommandGoal()
        g.command.position = float(position)
        g.command.max_effort = float(max_effort)

        if not action_client.wait_for_server(rospy.Duration(5)):
            raise ServiceException('connection to grippercommand action'
                                   ' server with name: ' + action_name +
                                   ' could not be established')

        run_action = self._generate_action_executor(action_client)

        with LeaseBaseLock([action_name]):
            response = await run_action(g)

        if not response:
            raise RuntimeError('Unexpected result received by'
                               ' gripper command action: ' +
                               action_name)

        return MoveGripperResult.from_gripper_command_action_result(response.result())

    async def wsg_gripper_command(self, action_name, command, width, speed,
                                  max_effort, stop_on_block=True):

        """
        Controls a WeissWsg gripper via the the specific action WsgCommandAction

        Parameters
        ----------
        action_name : str convertable
            Name of the action to control a specific gripper
        command : WsgCommand
            Specifies which kind of action should be performed
        width : float convertable
            Requested position of the gripper in meter
        speed : float convetable
            Requested speed in m/s
        max_effort : float convertable
            Force which should be applied

        Results
        -------
        action_response : MoveGripperResult
            Result of the action execution

        Raises
        ------
        TypeError
            If inputs are not convertable to specified types
            or if command is no of type WsgCommand
        ServiceError
            If action server is not available
        RuntimeError
            If action returns unexpected result
        """

        from wsg_50_common.msg import CommandAction, CommandGoal

        action_name = str(action_name)

        if not isinstance(command, WsgCommand):
            raise TypeError('command is not of expected type WsgCommand')

        action_client = actionlib.SimpleActionClient(action_name,
                                                     CommandAction)

        if not action_client.wait_for_server(rospy.Duration(5)):
            raise ServiceException('connection to wsg gripper action'
                                   ' server with name: ' + action_name +
                                   ' could not be established')

        g = CommandGoal()
        g.command.command_id = command.value
        g.command.width = float(width)
        g.command.speed = float(speed)
        g.command.force = float(max_effort)
        g.command.stop_on_block = bool(stop_on_block)

        run_action = self._generate_action_executor(action_client)

        with LeaseBaseLock([action_name]):
            response = await run_action(g)

        if not response:
            raise RuntimeError('Unexpected result received by'
                               ' wsg gripper command action: ' +
                               action_name)

        return WsgResult.from_wsg_command_action_result(response.result())

    def _generate_action_executor(self, action):

        async def run_action(goal):
            loop = asyncio.get_event_loop()
            action_done = loop.create_future()

            def done_callback(goal_status, result):
                status = ActionLibGoalStatus(goal_status)
                if status != ActionLibGoalStatus.SUCCEEDED:
                    try:
                        reason = MotionActionResult(result.result)
                        print('action end unsuccessfully with'
                              ' state: {}, reason: {}'.format(status, reason))
                    except AttributeError:
                        print('action end unsuccessfully with'
                              ' state: {}'.format(status))

                loop.call_soon_threadsafe(action_done.set_result, result)

            try:
                action.send_goal(goal, done_cb=done_callback)
                await action_done
            except (asyncio.CancelledError, ServiceException) as exc:
                print('Cancel goal because of: {}'.format(exc))
                action.cancel_goal()
                await action_done
                print('done2')

            return action_done

        return run_action
