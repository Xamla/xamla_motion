import asyncio
from typing import Callable, Dict, List, Tuple

import actionlib
import rospy
from rosgardener_msgs.msg import *
from rosgardener_msgs.srv import *
from xamla_motion.utility import ROSNodeSteward
from xamla_motion.xamla_motion_exceptions import ServiceException
from xamlamoveit_msgs.msg import *
from xamlamoveit_msgs.srv import *

from .data_types import ErrorCodes, SteppedMotionState
from .motion_service import SteppedMotionClient, generate_action_executor


class RobotChatClient(object):
    """
    Client to communicate with ROSVITA RoboChat
    """

    def __init__(self):
        """
        Initialize RobotChatClient

        Returns
        -------
        RobotChatClient
            Return an instance of RobotChatClient

        Raises
        ------
        ServiceException:
            If connection to ROSVITA RoboChat could
            not be established 
        """
        self.__robochat_channel_service_name = '/robochat/channel_command'
        self.__robochat_message_service_name = '/robochat/message_command'
        self.__robochat_query_action_name = '/robochat/query'
        self.__ros_node_steward = ROSNodeSteward()

        try:
            self.__channel_command_service = rospy.ServiceProxy(
                self.__robochat_channel_service_name,
                SetChannelCommand)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service with name: ' +
                                   self.__robochat_channel_service_name +
                                   ' could not be established') from exc

        try:
            self.__channel_message_service = rospy.ServiceProxy(
                self.__robochat_message_service_name,
                SetMessageCommand)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service with name: ' +
                                   self.__robochat_message_service_name +
                                   ' could not be established') from exc

    def _call_message_command(self, channel_name: str, command: str, message_body: str, message_id: str = None, arguments: List[str] = None) -> str:
        request = SetMessageCommandRequest()
        request.command.header.channel_name = channel_name
        request.command.header.command = command
        request.command.message_id = message_id or ''
        request.command.message_body = message_body or ''

        if arguments:
            request.command.header.arguments = arguments

        try:
            responds = self.__channel_message_service(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service call for query'
                                   ' message command'
                                   ' failed, abort') from exc

        return responds.response.message_id

    def _call_channel_command(self, name: str, command: str, arguments: List[str] = None) -> None:
        request = SetChannelCommandRequest()
        request.command.channel_name = name
        request.command.command = command
        if arguments:
            request.command.arguments = arguments

        try:
            self.__channel_command_service(request)
        except rospy.ServiceException as exc:
            raise ServiceException('service call for query'
                                   ' channel command'
                                   ' failed, abort') from exc

    def create_chat(self, name: str, topic: str, backlog: int = 1000):
        """
        Create a chat

        Parameters
        ----------
        name : str
            The name of the chat
        topic : str
            A topic
        backlog : int
            The size of the backlog

        Raises
        ------
        ServiceException:
            If service call to ROSVITA RoboChat fails 
        """
        self._call_channel_command(name, 'create', [topic, str(backlog)])

    def delete_chat(self, name: str):
        """
        Delete a chat

        Parameters
        ----------
        name : str
            Name of the chat to be deleted

        Raises
        ------
        ServiceException:
            If service call to ROSVITA RoboChat fails 
        """
        self._call_channel_command(name, 'delete')

    def clear_chat(self, name: str):
        """
        Clear a chat

        Parameters
        ----------
        name : str
            Name of the chat to be cleared

        Raises
        ------
        ServiceException:
            If service call to ROSVITA RoboChat fails 
        """
        self._call_channel_command(name, 'clear')

    def list_chats(self, name: str):
        """
        List chat

        Parameters
        ----------
        name : str
            Name of the chat

        Raises
        ------
        ServiceException:
            If service call to ROSVITA RoboChat fails 
        """
        self._call_channel_command(name, 'list')

    def set_topic_chat(self, name: str, topic: str):
        """
        Sets the the topic of a chat

        Parameters
        ----------
        name : str
            Name of the chat
        topic : str
            The topic name

        Raises
        ------
        ServiceException:
            If service call to ROSVITA RoboChat fails 
        """
        self._call_channel_command(name, 'set_topic', [topic])

    def create_text_message(self, channel_name: str, text: str, disposition: str):
        """
        Create a text message

        Parameters
        ----------
        channel_name : str
            Name of a channel
        text : str
            The text of the message

        Raises
        ------
        ServiceException:
            If service call to ROSVITA RoboChat fails 
        """
        return self._call_message_command(channel_name, 'add', text, None, [disposition])

    def delete_text_message(self, channel_name: str, message_id: str):
        """
        Delete a text message

        Parameters
        ----------
        channel_name : str
            Name of a channel
        message_id : str
            The id of the message to be deleted

        Raises
        ------
        ServiceException:
            If service call to ROSVITA RoboChat fails 
        """
        return self._call_message_command(channel_name, 'remove', None, message_id)

    def update_text_message(self, channel_name: str, message_id: str, text: str):
        """
        Update the content of a text message

        Parameters
        ----------
        channel_name : str
            Name of a channel
        message_id : str
            The id of the message to be updated
        text : str
            The updated text

        Raises
        ------
        ServiceException:
            If service call to ROSVITA RoboChat fails 
        """
        return self._call_message_command(channel_name, 'update', text, message_id)

    async def query_user_interaction(self, channel_name: str, command: str, arguments: List[str] = None, message_id: str = None, message_body: str = None):
        """
        Queries the users interaction with Ros asynchronously

        Parameters
        ----------
        channel_name : str
            Name of the channel
        command : str
            The command to be executed
        arguments : List[str
            ]List of arguments of the command
        message_id : str
            The id of the message
        message_body : str
            The message content

        Returns
        -------
        result
            Result of the query as a instance of rosgardener.RobochatQueryResult.

        Raises
        ------
        ServiceException:
            If action call to ROSVITA RoboChat fails 
        """
        query_action = actionlib.SimpleActionClient(self.__robochat_query_action_name,
                                                    RobochatQueryAction)

        if not query_action.wait_for_server(rospy.Duration(5)):
            raise ServiceException('connection to robot chat query action'
                                   ' server could not be established')

        query_action = generate_action_executor(query_action)
        goal = RobochatQueryGoal()
        goal.command.header.channel_name = channel_name
        goal.command.header.command = command
        if arguments:
            goal.command.header.arguments = arguments

        if message_id:
            goal.command.message_id = message_id
        if message_body:
            goal.command.message_body = message_body

        return await query_action(goal)


class RobotChatSteppedMotion(object):
    """
    Class to run a supervised motion from python 
    with interaction dialog in ROSVITA
    """

    def __init__(self, robot_chat: RobotChatClient, stepped_client: SteppedMotionClient,
                 move_group_name: str):
        """
        Initialization of RoboChatSteppedMotion

        Parameters
        ----------
        robot_chat : RosRoboChatClient
            RoboChat client to communicate with ROSVITA supervised motion dialog
        stepped_client : SteppedMotionClient
            Client which controlles the supervised motion
        move_group_name : str
            Name of the move group which executes the motion

        Returns
        -------
        RobotChatSteppedMotion
            Returns an instance of RobotChatSteppedMotion

        Raises
        ------
        TypeError : type mismatch
            If robot chat is not of expected type RobotChatClient
            If stepped_client is not of expected type SteppedMotionClient
            If move_group_name is not of exptected type str
        """

        if not isinstance(robot_chat, RobotChatClient):
            raise TypeError('robot_chat is not of expected'
                            ' type RobotChatClient')

        if not isinstance(stepped_client, SteppedMotionClient):
            raise TypeError('stepped_client is not of expected'
                            ' type SteppedMotionClient')

        self.robot_chat = robot_chat
        self.stepped_client = stepped_client
        self.move_group_name = str(move_group_name)

    async def handle_stepwise_motions(self):
        """
        Start supervised motion and ROSVITA supervised motion dialog

        Raises
        ------
        ServiceException
            If stepped motion ends not successful
        """
        channel_name = "MotionDialog"
        topic = "SteppedMotions"
        disposition = "SteppedMotion"

        self.robot_chat.create_chat(channel_name, topic)
        message_body = self._create_message(str(self.stepped_client.goal_id.id),
                                            self.move_group_name, 0.0)
        message_id = self.robot_chat.create_text_message(
            channel_name, message_body, disposition)

        try:
            task_update = asyncio.ensure_future(
                self._update_progress(channel_name, message_id))
            await self.stepped_client.action_done_future
        except ServiceException as exc:
            if (exc.error_code != ErrorCodes.SUCCESS and
                    exc.error_code != ErrorCodes.PREEMPTED):
                raise ServiceException('Robot chat stepped motion ends '
                                       'not successful') from exc
        finally:
            self.robot_chat.delete_text_message(channel_name, message_id)
            task_update.cancel()

    async def _update_progress(self, channel_name: str, message_id: str):
        lastProgress = 0.0
        run = True
        while run:
            await asyncio.sleep(0.05)
            if self.stepped_client.state:
                state = self.stepped_client.state
                if (state.progress != lastProgress):
                    lastProgress = state.progress
                    message_body = self._create_message(str(self.stepped_client.goal_id.id),
                                                        self.move_group_name, lastProgress)
                    message_id = self.robot_chat.update_text_message(
                        channel_name, message_id, message_body)
                    run = state.error_code == ErrorCodes.PROGRESS

    def _create_message(self, goal_id: str, move_group_name: str, progress: float) -> str:
        my_message = '"GoalId" : "{}", "MoveGroupName" : "{}", "Progress" : {}'
        return '{' + my_message.format(goal_id, move_group_name, progress) + '}'
