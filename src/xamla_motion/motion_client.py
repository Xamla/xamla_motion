from typing import Dict, List, Tuple, Union

from xamla_motion.data_types import (CartesianPath, JointPath, JointSet,
                                     JointTrajectory, JointValues,
                                     PlanParameters, Pose)
from xamla_motion.v2.motion_client import EndEffector as NewEndEffector
from xamla_motion.v2.motion_client import MoveGroup as NewMoveGroup
from xamla_motion.motion_service import MotionService, SteppedMotionClient

class MoveGroup(NewMoveGroup):

    """
    Class with encapsulate move functionality for a specific move group

    Methods
    -------
    set_default_end_effector(end_effector_name: str)
        Set one of the end effector from the list of available ones as default
    get_end_effector(name: Union[None, str]) -> EndEffector
        Get the end effector specified by name or the default end effector
    get_current_joint_states() -> JointStates
        Returns the current joint states of the move group joints
    get_current_joint_positions() -> JointValues
        Returns the current joint positions of the move group joints
    plan_move_joints(
            target: Union[JointValues, JointPath], velocity_scaling: Union[None, float]=None,
            collision_check: Union[None, bool]=None, max_deviation: Union[None, float]=None,
            acceleration_scaling: Union[None, float]=None
        ) -> Tuple[JointTrajectory, PlanParameters]
        Plans a trajectory from current state to target joint positions
    plan_move_joints_collision_free(
            target: Union[JointValues, JointPath], velocity_scaling: Union[None, float]=None,
            collision_check: Union[None, bool]=None, max_deviation: Union[None, float]=None,
            acceleration_scaling: Union[None, float]=None
        ) -> Tuple[JointTrajectory, PlanParameters]
        Plans a collision free trajectory from current to target joint positions.
    move_joints(
            target: Union[JointValues, JointPath], velocity_scaling: Union[None, float]=None,
            collision_check: Union[None, bool]=None, max_deviation: Union[None, float]=None,
            acceleration_scaling: Union[None, float]=None)
        Asynchronous plan and execute joint trajectory
    move_joints_supervised(
            target: Union[JointValues, JointPath], velocity_scaling: Union[None, float]=None,
            collision_check: Union[None, bool]=None, max_deviation: Union[None, float]=None,
            acceleration_scaling: Union[None, float]=None
        ) -> SteppedMotionClient
        Plan joint trajectory and creates a supervised executor
        Create MoveJointsOperation for target joint positions
    move_joints_collision_free(
            target: Union[JointValues, JointPath], velocity_scaling: Union[None, float]=None,
            collision_check: Union[None, bool]=None, max_deviation: Union[None, float]=None,
            acceleration_scaling: Union[None, float]=None)
        Asynchronous plan and execute collision free joint trajectory
    move_joints_collision_free_supervised(
            target: Union[JointValues, JointPath], velocity_scaling: Union[None, float]=None,
            collision_check: Union[None, bool]=None, max_deviation: Union[None, float]=None,
            acceleration_scaling: Union[None, float]=None
        ) -> SteppedMotionClient
        Plan collision free joint trajectory and creates a supervised executor
    """

    def __init__(self, move_group_name: Union[None, str] = None,
                 end_effector_name: Union[None, str] = None, motion_service=None):
        """
        Initialize MoveGroup class

        The initialization process is possible with different settings:

        move_group_name and end_effector_name are None:
            In this case the first available move group and its
            first end effector is selected automatically. If no
            move group or end_effector is available exception is
            raised.
        move_group_name is defined and end_effector_name is none:
            In this case the initialization routine tries to find
            the move group with requested name. If it is available
            create a instance for this move group and the first
            available end effector as default end effector. Else
            an exceptions is raised.
        move_group is None and end_effector_name is defined:
            In this case the initialization routine tries to find
            the move group with requested end effector. If it is
            available create a instance for this move group and the
            requested end effector as default end effector. Else
            an exceptions is raised.
        move_group is defined and also end_effector_name is defined:
            In this case the initialization routine tries to find
            the requested move group with requested end effector.
            If it is available create a instance for this move group
            and the requested end effector as default end effector. Else
            an exceptions is raised.

        Parameters
        ----------
        move_group_name : Union[None, str]
            If defined name of the move group which this
            instance should represent
        end_effector_name : Union[None, str]
            If defined name of the end_effector which should
            be used as default end_effector
        motion_service : Union[None, MotionService]
            An instance of MotionService which is used to
            communicate with the motion server if None a
            new instance of MotionService is created

        Raises
        ------
        TypeError
            If motion_service is not of type MotionService
            or If the other inputs are not str convertable
        ServiceError
            If necessary services to query available move
            groups and end effectors are not available
        RuntTimeError
            If requested move group or end effector not
            exist or are not available
        """

        if not motion_service:
            motion_service = MotionService()

        if not isinstance(motion_service, MotionService):
            raise TypeError('motion_service is not of'
                            ' expected type MotionService')

        self.__m_service = motion_service

        groups = self.__m_service.query_available_move_groups()
        if not groups:
            raise RuntimeError('no move groups available')

        if end_effector_name:
            end_effector_name = str(end_effector_name)
        if move_group_name:
            move_group_name = str(move_group_name)

        if not end_effector_name and not move_group_name:
            move_group_name = groups[0].name
            try:
                end_effector_name = groups[0].end_effector_names[0]
            except IndexError:
                end_effector_name = None
            details = groups[0]

        elif end_effector_name and not move_group_name:
            try:
                details = next(g for g in groups
                               if any([end_effector_name in
                                       g.end_effector_names]))
            except StopIteration:
                raise RuntimeError('it exist no move group with'
                                   ' end effector: '
                                   + end_effector_name)
            move_group_name = details.name

        elif not end_effector_name and move_group_name:
            try:
                details = next(g for g in groups
                               if g.name == move_group_name)
            except StopIteration:
                raise RuntimeError('it exist no move group with'
                                   ' name: ' + move_group_name)

            try:
                end_effector_name = details.end_effector_names[0]
            except IndexError:
                end_effector_name = None

        else:
            try:
                details = next(g for g in groups
                               if any([end_effector_name in
                                       g.end_effector_names]))
            except StopIteration:
                raise RuntimeError('move group: ' + move_group_name +
                                   ' with end effector: '
                                   + end_effector_name +
                                   ' not exists')

        self.__name = move_group_name
        self.__details = details
        self.__joint_set = self.__details.joint_set
        p = self.__m_service.create_plan_parameters(move_group_name,
                                                    self.__joint_set,
                                                    sample_resolution=0.05,
                                                    collision_check=True)
        self.__plan_parameters = p

        names = self.__details.end_effector_names

        if names:
            link_names = self.__details.end_effector_link_names
            self.__end_effectors = {name: EndEffector(self,
                                                      name,
                                                      link_names[i])
                                    for i, name in enumerate(names)}

            self.set_default_end_effector(end_effector_name)
        else:
            self.__end_effectors = {}
            self.__selected_end_effector = None
            self.__task_space_plan_parameters = None

        self.__velocity_scaling = 1.0
        self.__acceleration_scaling = 1.0

    def plan_move_joints(self,
                         target: Union[JointValues, JointPath],
                         velocity_scaling: Union[None, float] = None,
                         collision_check: Union[None, bool] = None,
                         max_deviation: Union[None, float] = None,
                         acceleration_scaling: Union[None, float] = None
                         ) -> Tuple[JointTrajectory, PlanParameters]:
        """
        TODO: Also test if this behaves as intended

        Plans a trajectory from current state to target joint positions
        Parameters
        ----------
        target : Union[JointValues, JointPath]
            Target joint positions or target JointPath
        velocity_scaling : Union[None, float]
            Scaling factor which is applied on the maximal
            possible joint velocities
        collision_check : Union[None, bool]
            If true the trajectory planing try to plan a
            collision free trajectory and before executing
            a trajectory a collision check is performed
        max_deviation : Union[None, float]
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : Union[None, float]
            Scaling factor which is applied on the maximal
            possible joint accelerations

        Returns
        -------
        trajectory, parameters : Tuple[JointTrajectory, PlanParameters]
            Returns a tuple where the firt argument is the planed
            trajectory and the second argument the parameters which are
            used for it

        Raises
        ------
        TypeError
            If target is not one of types JointValues, JointPath
            If all other inputs are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        ServiceError
            If underlying services from motion server are not available
            or finish not successfully
        """
        move_joints_op = super(MoveGroup, self).move_joints(target=target,
                                                            velocity_scaling=velocity_scaling,
                                                            collision_check=collision_check,
                                                            max_deviation=max_deviation,
                                                            acceleration_scaling=acceleration_scaling)
        plan = move_joints_op.plan()
        return plan.trajectory, plan.parameters

    def plan_move_joints_collision_free(self,
                                        target: Union[JointValues, JointPath],
                                        velocity_scaling: Union[None, float] = None,
                                        collision_check: Union[None, bool] = None,
                                        max_deviation: Union[None, float] = None,
                                        acceleration_scaling: Union[None,
                                                                    float] = None
                                        ) -> Tuple[JointTrajectory, PlanParameters]:
        """
        TODO: Also test if this behaves as intended
        TODO: Added collision check
        Plans a collision free trajectory from current to target joint positions

        Parameters
        ----------
        target : Union[JointValues, JointPath]
            Target joint positions or joint path
        velocity_scaling : Union[None, float]
            Scaling factor which is applied on the maximal
            possible joint velocities
        max_deviation : Union[None, float]
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        collision_check : Union[None, bool]
            If true the trajectory planing try to plan a
            collision free trajectory and before executing
            a trajectory a collision check is performed
        acceleration_scaling : Union[None, float]
            Scaling factor which is applied on the maximal
            possible joint accelerations

        Returns
        -------
        trajectory, parameters : Tuple[JointTrajectory, PlanParameters]
            Returns a tuple where the firt argument is the planed
            trajectory and the second argument the parameters which are
            used for it

        Raises
        ------
        TypeError
            If target is not one of types JointValues, JointPath
            If all other inputs are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        ServiceError
            If underlying services from motion server are not available
            or finish not successfully
        """
        move_joints_op = super(MoveGroup, self).move_joints_collision_free(
            target=target,
            velocity_scaling=velocity_scaling,
            collision_check=collision_check,
            max_deviation=max_deviation,
            acceleration_scaling=acceleration_scaling)
        plan = move_joints_op.plan()
        return plan.trajectory, plan.parameters

    async def move_joints(self,
                          target: Union[JointValues, JointPath],
                          velocity_scaling: Union[None, float] = None,
                          collision_check: Union[None, bool] = None,
                          max_deviation: Union[None, float] = None,
                          acceleration_scaling: Union[None, float] = None
                          ):
        """
        Asynchronous plan and execute joint trajectory from joint space input

        Parameters
        ----------
        target : Union[JointValues, JointPath]
            Target joint positions or joint path
        velocity_scaling : Union[None, float]
            Scaling factor which is applied on the maximal
            possible joint velocities
        collision_check : Union[None, bool]
            If true the trajectory planing try to plan a
            collision free trajectory and before executing
            a trajectory a collision check is performed
        max_deviation : Union[None, float]
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : Union[None, float]
            Scaling factor which is applied on the maximal
            possible joint accelerations

        Raises
        ------
        TypeError
            If target is not one of types JointValues, JointPath
            If all other inputs are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        ServiceError
            If underlying services from motion server are not available
            or finish not successfully
        """

        move_joints_op = super(MoveGroup, self).move_joints(
            target=target,
            velocity_scaling=velocity_scaling,
            collision_check=collision_check,
            max_deviation=max_deviation,
            acceleration_scaling=acceleration_scaling)
        plan = move_joints_op.plan()
        await plan.execute_async()

    def move_joints_supervised(self,
                               target: Union[JointValues, JointPath],
                               velocity_scaling: Union[None, float] = None,
                               collision_check: Union[None, bool] = None,
                               max_deviation: Union[None, float] = None,
                               acceleration_scaling: Union[None, float] = None
                               ) -> SteppedMotionClient:
        """
        Plan joint trajectory and creates a supervised executor

        Parameters
        ----------
        target : Union[JointValues, JointPath]
            Target joint positions or joint path
        velocity_scaling : Union[None, float]
            Scaling factor which is applied on the maximal
            possible joint velocities
        collision_check : Union[None, bool]
            If true the trajectory planing try to plan a
            collision free trajectory and before executing
            a trajectory a collision check is performed
        max_deviation : Union[None, float]
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : Union[None, float]
            Scaling factor which is applied on the maximal
            possible joint accelerations

        Returns
        -------
        executor : SteppedMotionClient
            Executor for supervised execution of trajectory

        Raises
        ------
        TypeError
            If target is not one of types JointValues, JointPath
            If all other inputs are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        ServiceError
            If underlying services from motion server are not available
            or finish not successfully
        """

        move_joints_op = super(MoveGroup, self).move_joints(
            target=target, velocity_scaling=velocity_scaling,
            collision_check=collision_check,
            max_deviation=max_deviation,
            acceleration_scaling=acceleration_scaling)
        plan = move_joints_op.plan()
        return plan.execute_supervised()

    async def move_joints_collision_free(self,
                                         target: Union[JointValues, JointPath],
                                         velocity_scaling: Union[None, float] = None,
                                         collision_check: Union[None, bool] = None,
                                         max_deviation: Union[None, float] = None,
                                         acceleration_scaling: Union[None,
                                                                     float] = None
                                         ):
        """
        Asynchronous plan and execute collision free joint trajectory from joint space input

        Parameters
        ----------
        target : Union[JointValues, JointPath]
            Target joint positions or joint path
        velocity_scaling : Union[None, float]
            Scaling factor which is applied on the maximal
            possible joint velocities
        max_deviation : Union[None, float]
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : Union[None, float]
            Scaling factor which is applied on the maximal
            possible joint accelerations

        Raises
        ------
        TypeError
            If target is not one of types JointValues, JointPath
            If all other inputs are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        ServiceError
            If underlying services from motion server are not available
            or finish not successfully
        """

        move_joints_op = super(MoveGroup, self).move_joints_collision_free(
            target=target,
            velocity_scaling=velocity_scaling,
            collision_check=collision_check,
            max_deviation=max_deviation,
            acceleration_scaling=acceleration_scaling)
        plan = move_joints_op.plan()
        await plan.execute_async()

    def move_joints_collision_free_supervised(self,
                                              target: Union[JointValues, JointPath],
                                              velocity_scaling: Union[None, float] = None,
                                              collision_check: Union[None, bool] = None,
                                              max_deviation: Union[None, float] = None,
                                              acceleration_scaling: Union[None,
                                                                          float] = None
                                              ) -> SteppedMotionClient:
        """
        plan collision free joint trajectory and creates a supervised executor

        Parameters
        ----------
        target : Union[JointValues, JointPath]
            Target joint positions or joint path
        velocity_scaling : Union[None, float]
            Scaling factor which is applied on the maximal
            possible joint velocities
        max_deviation : Union[None, float]
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : Union[None, float]
            Scaling factor which is applied on the maximal
            possible joint accelerations

        Returns
        -------
        executor : SteppedMotionClient
            Executor for supervised execution of trajectory

        Raises
        ------
        TypeError
            If target is not one of types JointValues, JointPath
            If all other inputs are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        ServiceError
            If underlying services from motion server are not available
            or finish not successfully
        """

        move_joints_op = super(MoveGroup, self).move_joints_collision_free(
            target=target,
            velocity_scaling=velocity_scaling,
            collision_check=collision_check,
            max_deviation=max_deviation,
            acceleration_scaling=acceleration_scaling)
        plan = move_joints_op.plan()
        return plan.execute_supervised()


class EndEffector(NewEndEffector):

    """
    Class with encapsulate move functionality for a specific end effector

    Methods
    -------
    from_end_effector_name(end_effector_name: str) -> EndEffector
        Creates an instance of MoveGroup and select the correct instance of EndEffector
    get_current_pose() -> Pose
        Returns the current pose of the end effector
    compute_pose(joint_values: JointValues) -> Pose
        compute pose from joint values / configuration
    inverse_kinematics(
            poses: Union[Pose, CartesianPath], collision_check: Union[None, bool], seed: Union[None, JointValues],
            timeout:  Union[None, datatime.timedelta], const_seed: bool, attempts: int
        ) -> JointValues:
        inverse kinematic solutions for one pose
    inverse_kinematics_many(
            poses: Union[Pose, CartesianPath], collision_check: Union[None, bool], seed: Union[None, JointValues],
            timeout:  Union[None, datatime.timedelta], const_seed: bool, attempts: int
        ) -> IkResults:
        inverse kinematic solutions for many poses
    move_poses(
            target: Union[Pose, CartesianPath], seed: Union[None, JointValues]=None,
            velocity_scaling: Union[None, float]=None, collision_check: Union[None, bool]=None,
            max_deviation: Union[None, float]=None, acceleration_scaling: Union[None, float]=None)
        Asynchronous plan and execute trajectory from task space input
    move_poses_supervised(
            target: Union[Pose, CartesianPath], seed: Union[None, JointValues]=None,
            velocity_scaling: Union[None, float]=None, collision_check: Union[None, bool]=None,
            max_deviation: Union[None, float]=None, acceleration_scaling: Union[None, float]=None
        ) -> SteppedMotionClient
        Plan trajectory from task space input and create executor
    move_poses_collision_free(
            target: Union[Pose, CartesianPath], seed: Union[None, JointValues]=None,
            velocity_scaling: Union[None, float]=None, collision_check: Union[None, bool]=None,
            max_deviation: Union[None, float]=None, acceleration_scaling: Union[None, float]=None)
        Asynchronous plan and execute collision free trajectory from task space input
    move_poses_collision_free_supervised(
            target: Union[Pose, CartesianPath], seed: Union[None, JointValues]=None,
            velocity_scaling: Union[None, float]=None, collision_check: Union[None, bool]=None,
            max_deviation: Union[None, float]=None, acceleration_scaling: Union[None, float]=None
        ) -> SteppedMotionClient
        Plan collision free trajectory from task space input and create executor
    plan_poses_linear(
            target: Union[Pose, CartesianPath], velocity_scaling: Union[None, float]=None,
            collision_check: Union[None, bool]=None, max_deviation: Union[None, float]=None,
            acceleration_scaling: Union[None, float]=None
        ) -> Tuple[JointTrajectory, PlanParameters]
        Plans a trajectory with linear movements from task space input
    move_poses_linear(
            target: Union[Pose, CartesianPath], velocity_scaling: Union[None, float]=None,
            collision_check: Union[None, bool]=None, max_deviation: Union[None, float]=None,
            acceleration_scaling: Union[None, float]=None)
        Plans and executes a trajectory with linear movements from task space input
    move_poses_linear_supervised(
            target: Union[Pose, CartesianPath], velocity_scaling: Union[None, float]=None,
            collision_check: Union[None, bool]=None, max_deviation: Union[None, float]=None,
            acceleration_scaling: Union[None, float]=None)
        Plans and executes a trajectory with linear movements from task space input
    """

    def __init__(self, move_group: MoveGroup, end_effector_name: str,
                 end_effector_link_name: str):
        """
        Initialize EndEffector class

        Parameters
        ----------
        move_group : MoveGroup
            Instance of move group where the endeffector belongs
            to
        end_effector_name : str_convertable
            If defined name of the end_effector which should
            for which a instance of EndEffector is created
            and which is selected as default for the move
            group
        end_effector_link_name : str convertable
            name of the end effector link

        Raises
        ------
        TypeError
            If move_group is not of type MoveGroup
            or If the other inputs are not str convertable
        ServiceError
            If necessary services to query available move
            groups and end effectors are not available
        RuntTimeError
            If requested move group or end effector not
            exist or are not available
        """

        if not isinstance(move_group, MoveGroup):
            raise TypeError('move_group is not of expected'
                            ' type MoveGroup')

        self.__name = str(end_effector_name)
        self.__move_group = move_group
        self.__link_name = str(end_effector_link_name)
        self.__m_service = move_group.motion_service

    @staticmethod
    def from_end_effector_name(end_effector_name: str) -> "EndEffector":
        """
        Creates an instance of MoveGroup and select the correct instance of EndEffector

        Parameters
        ----------
        end_effector_name : str convertable
            Name of the end_effector for which a instance of EndEffector
            should be created

        Returns
        -------
        EndEffector
            Instance of EndEffector for end effector with the name
            specified by end_effector name (instance is also hold
            by the created move_group)

        Raises
        ------
        TypeError
            If end_effector_name ist not convertable to str
        RuntimeError
            If it is not possible to create an instance of
            EndEffector because the end effector specified
            by end_effector_name is not available
        """

        end_effector_name = str(end_effector_name)

        try:
            move_group = MoveGroup(None, end_effector_name)
        except RuntimeError as exc:
            raise RuntimeError('end effector with name: ' +
                               end_effector_name +
                               ' is not available') from exc

        return move_group.get_end_effector()

    async def move_poses(self, target: Union[Pose, CartesianPath],
                         seed: Union[None, JointValues] = None,
                         velocity_scaling: Union[None, float] = None,
                         collision_check: Union[None, bool] = None,
                         max_deviation: Union[None, float] = None,
                         acceleration_scaling: Union[None, float] = None):
        move_joints_op = super(EndEffector, self).move_cartesian(
            target=target,
            seed=seed,
            velocity_scaling=velocity_scaling,
            collision_check=collision_check,
            max_deviation=max_deviation,
            acceleration_scaling=acceleration_scaling)
        plan = move_joints_op.plan()
        await plan.execute_async()

    def move_poses_supervised(self, target: (Pose, CartesianPath),
                              seed: (None, JointValues) = None,
                              velocity_scaling: Union[None, float] = None,
                              collision_check: Union[None, bool] = None,
                              max_deviation: Union[None, float] = None,
                              acceleration_scaling: Union[None, float] = None
                              ) -> SteppedMotionClient:

        move_joints_op = super(EndEffector, self).move_cartesian(
            target=target,
            seed=seed,
            velocity_scaling=velocity_scaling,
            collision_check=collision_check,
            max_deviation=max_deviation,
            acceleration_scaling=acceleration_scaling)
        plan = move_joints_op.plan()
        return plan.execute_supervised()

    async def move_poses_collision_free(self, target: Union[Pose, CartesianPath],
                                        seed: Union[None, JointValues] = None,
                                        velocity_scaling: Union[None, float] = None,
                                        collision_check: Union[None, bool] = None,
                                        max_deviation: Union[None, float] = None,
                                        acceleration_scaling: Union[None, float] = None):
        """
        Asynchronous plan and execute collision free trajectory from task space input
        Parameters
        ----------
        target : Union[Pose, CartesianPath]
            Target joint positions
        seed : Union[None, JointValues] (optional)
            Numerical seed to control joint configuration
        velocity_scaling : Union[None, float]  (optional)
            Scaling factor which is applied on the maximal
            possible joint velocities
        collision_check : Union[None, bool] (optional)
            If true the trajectory planing try to plan a
            collision free trajectory and before executing
            a trajectory a collision check is performed
        max_deviation : Union[None, float] (optional)
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : Union[None, float]  (optional)
            Scaling factor which is applied on the maximal
            possible joint accelerations
        Raises
        ------
        TypeError
            If target is not one of types Union[Pose, CartesianPath]
            If seed is defined and not of type JointValues
            If all other inputs are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        ServiceError
            If underlying services from motion server are not available
            or finish not successfully
        """

        move_joints_op = super(EndEffector, self).move_cartesian_collision_free(
            target=target,
            seed=seed,
            velocity_scaling=velocity_scaling,
            collision_check=collision_check,
            max_deviation=max_deviation,
            acceleration_scaling=acceleration_scaling)
        plan = move_joints_op.plan()
        await plan.execute_async()

    def move_poses_collision_free_supervised(self, target: Union[Pose, CartesianPath],
                                             seed: Union[None, JointValues] = None,
                                             velocity_scaling: Union[None, float] = None,
                                             collision_check: Union[None, bool] = None,
                                             max_deviation: Union[None, float] = None,
                                             acceleration_scaling: Union[None, float] = None
                                             ) -> SteppedMotionClient:
        """
        Plan collision free trajectory from task space input and create executor

        Parameters
        ----------
        target : Union[Pose, CartesianPath]
            Target joint positions
        seed : Union[None, JointValues] (optional)
            Numerical seed to control joint configuration
        velocity_scaling : Union[None, float]  (optional)
            Scaling factor which is applied on the maximal
            possible joint velocities
        collision_check : Union[None, bool] (optional)
            If true the trajectory planing try to plan a
            collision free trajectory and before executing
            a trajectory a collision check is performed
        max_deviation : Union[None, float] (optional)
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : Union[None, float]  (optional)
            Scaling factor which is applied on the maximal
            possible joint accelerations

        Returns
        -------
        executor : SteppedMotionClient
            Executor for supervised execution of collision free trajectory

        Raises
        ------
        TypeError
            If target is not one of types Union[Pose, CartesianPath]
            If seed is defined and not of type JointValues
            If all other inputs are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        ServiceError
            If underlying services from motion server are not available
            or finish not successfully
        """

        move_joints_op = super(EndEffector, self).move_cartesian_collision_free(
            target=target,
            seed=seed,
            velocity_scaling=velocity_scaling,
            collision_check=collision_check,
            max_deviation=max_deviation,
            acceleration_scaling=acceleration_scaling)
        plan = move_joints_op.plan()
        return plan.execute_supervised()

    def plan_poses_linear(self, target: Union[Pose, CartesianPath],
                          seed: Union[None, JointValues] = None,
                          velocity_scaling: Union[None, float] = None,
                          collision_check: Union[None, bool] = None,
                          max_deviation: Union[None, float] = None,
                          acceleration_scaling: Union[None, float] = None
                          ) -> Tuple[JointTrajectory, PlanParameters]:
        """
        Plans a trajectory with linear movements from task space input

        Parameters
        ----------
        target : Union[Pose, CartesianPath]
            Target joint positions
        velocity_scaling : Union[None, float]
            Scaling factor which is applied on the maximal
            possible joint velocities
        collision_check : Union[None, bool]
            If true the trajectory planing try to plan a
            collision free trajectory and before executing
            a trajectory a collision check is performed
        max_deviation : Union[None, float]
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : Union[None, float]
            Scaling factor which is applied on the maximal
            possible joint accelerations

        Returns
        -------
        Tuple[JointTrajectory, PlanParameters]
            A tuple containing the trajectory and the plan parameters

        Raises
        ------
        TypeError
            If target is not one of types Union[Pose, CartesianPath]
            If all other inputs are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        ServiceError
            If underlying services from motion server are not available
            or finish not successfully
        """

        move_joints_op = super(EndEffector, self).move_cartesian_linear(
            target=target,
            seed=seed,
            velocity_scaling=velocity_scaling,
            collision_check=collision_check,
            max_deviation=max_deviation,
            acceleration_scaling=acceleration_scaling)
        plan = move_joints_op.plan()
        return plan.trajectory, plan.parameters

    async def move_poses_linear(self, target: Union[Pose, CartesianPath],
                                seed: Union[None, JointValues]=None,
                                velocity_scaling: Union[None, float]=None,
                                collision_check: Union[None, bool]=None,
                                max_deviation: Union[None, float]=None,
                                acceleration_scaling: Union[None, float]=None):
        """
        Plans and executes a trajectory with linear movements from task space input

        Parameters
        ----------
        target : Union[Pose, CartesianPath]
            Target joint positions
        velocity_scaling : Union[None, float]
            Scaling factor which is applied on the maximal
            possible joint velocities
        collision_check : Union[None, bool]
            If true the trajectory planing try to plan a
            collision free trajectory and before executing
            a trajectory a collision check is performed
        max_deviation : Union[None, float]
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : Union[None, float]
            Scaling factor which is applied on the maximal
            possible joint accelerations

        Raises
        ------
        TypeError
            If target is not one of types Union[Pose, CartesianPath]
            If all other inputs are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        ServiceError
            If underlying services from motion server are not available
            or finish not successfully
        """

        move_joints_op = super(EndEffector, self).move_cartesian_linear(
            target=target,
            seed=seed,
            velocity_scaling=velocity_scaling,
            collision_check=collision_check,
            max_deviation=max_deviation,
            acceleration_scaling=acceleration_scaling)
        plan = move_joints_op.plan()
        await plan.execute_async()

    def move_poses_linear_supervised(self, target: Union[Pose, CartesianPath],
                                     seed: Union[None, JointValues]=None,
                                     velocity_scaling: Union[None, float]=None,
                                     collision_check: Union[None, bool]=None,
                                     max_deviation: Union[None, float]=None,
                                     acceleration_scaling: Union[None, float]=None
                                     ) -> SteppedMotionClient:
        """
        Plans and executes a trajectory with linear movements from task space input supervised

        Parameters
        ----------
        target : Union[Pose, CartesianPath]
            Target joint positions
        velocity_scaling : Union[None, float]
            Scaling factor which is applied on the maximal
            possible joint velocities
        collision_check : Union[None, bool]
            If true the trajectory planing try to plan a
            collision free trajectory and before executing
            a trajectory a collision check is performed
        max_deviation : Union[None, float]
            Defines the maximal deviation from trajectory points
            when it is a fly-by-point in joint space
        acceleration_scaling : Union[None, float]
            Scaling factor which is applied on the maximal
            possible joint accelerations

        Raises
        ------
        TypeError
            If target is not one of types Union[Pose, CartesianPath]
            If all other inputs are not convertable to specified types
        ValueError
            If scaling inputs are not between 0.0 and 1.0
        ServiceError
            If underlying services from motion server are not available
            or finish not successfully
        """

        move_joints_op = super(EndEffector, self).move_cartesian_linear(
            target=target,
            seed=seed,
            velocity_scaling=velocity_scaling,
            collision_check=collision_check,
            max_deviation=max_deviation,
            acceleration_scaling=acceleration_scaling)
        plan = move_joints_op.plan()
        return plan.execute_supervised()
