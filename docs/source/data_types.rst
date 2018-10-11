.. role:: hidden
    :class: hidden-section

Data types
=========================

.. automodule:: xamla_motion.data_types
.. currentmodule:: xamla_motion.data_types

Overview
------------------------------------------

In order to give a better overview of the data types
which are defined by xamla_motion these are 
devided into five groups. 

The groups are robot and environment description types, 
joint space data types, task space data types and 
state/result data types

Robot and environment description types
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
==================================================================  ========================================================================================   
Data type                                                           Description                                         
==================================================================  ========================================================================================   
:class:`xamla_motion.data_types.JointSet`                           Manages a list of joint names which describes a robot or part of it
:class:`xamla_motion.data_types.EndEffectorDescription`             Describes an end effector of a robot
:class:`xamla_motion.data_types.MoveGroupDescription`               Describes a robot or a subset of it by move groups, end effectors and joints
:class:`xamla_motion.data_types.CollisionPrimitive`                 Defines and describe a primitive collision object
:class:`xamla_motion.data_types.CollisionObject`                    Defines a collision object which consists of a single primitive or an assembly of them
==================================================================  ========================================================================================


Joint space data types
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

==================================================================  ========================================================================================   
Data type                                                           Description                                         
==================================================================  ========================================================================================   
:class:`xamla_motion.data_types.JointValues`                        Manages respective joint values for a specific joint set
:class:`xamla_motion.data_types.JointLimits`                        Manages the joint limits for a specific set of joints
:class:`xamla_motion.data_types.JointStates`                        Can hold a complete JointState (pos, vel, eff) for a specific joint set
:class:`xamla_motion.data_types.JointPath`                          Describes a path by a list of joint configurations/values
:class:`xamla_motion.data_types.JointTrajectoryPoint`               Defines a point of a trajectory for a specific joint set
:class:`xamla_motion.data_types.JointTrajectory`                    Describes a trajectory by a list of trajectory points
:class:`xamla_motion.data_typesPlanParameters`                      Holds all constrains for joint space trajectory planning
==================================================================  ========================================================================================

Task space data types
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

==================================================================  ========================================================================================   
Data type                                                           Description                                         
==================================================================  ========================================================================================   
:class:`xamla_motion.data_types.Pose`                               Pose defined by three dimensional translation and rotation
:class:`xamla_motion.data_types.EndEffectorLimits`                  Holds hold task space / endeffector limits
:class:`xamla_motion.data_types.CartesianPath`                      Describes a path by a list of poses
:class:`xamla_motion.data_types.TaskSpacePlanParameters`            Holds all constrains for task space trajectory planning
==================================================================  ========================================================================================

State and result data types
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

==================================================================  ========================================================================================   
Data type                                                           Description                                         
==================================================================  ========================================================================================   
:class:`xamla_motion.data_types.MoveGripperResult`                  Represents the result of a common gripper action
:class:`xamla_motion.data_types.WsgResult`                          Respresents the result of a WSG gripper action
:class:`xamla_motion.data_types.IkResults`                          Holds result of a inverse kinematics query
:class:`xamla_motion.data_types.SteppedMotionState`                 Carries a state snapshot of a stepped motion operation
:class:`xamla_motion.data_types.JointValuesCollisions`              Represents a robot collision
==================================================================  ========================================================================================


Robot and environment description types
------------------------------------------

.. _jointset:

JointSet
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: JointSet
    :members:

.. _endeffectordescription:

EndEffectorDescription
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

.. autoclass:: EndEffectorDescription
    :members:

Details:

.. _movegroupdescription:

MoveGroupDescription
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: MoveGroupDescription
    :members:


.. _collisionprimitive:

CollisionPrimitive
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: CollisionPrimitive
    :members:

.. _collisionobject:

CollisionObject
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: CollisionObject
    :members:


Joint space data types
------------------------------------------

JointValues
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: JointValues
    :members:

JointLimits
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: JointLimits
    :members:

JointStates
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: JointStates
    :members:

JointPath
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: JointPath
    :members:

JointTrajectoryPoint
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: JointTrajectoryPoint
    :members:

JointTrajectory
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: JointTrajectory
    :members:

PlanParameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: PlanParameters
    :members:

Task space data types
------------------------------------------

Pose
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: Pose
    :members:

EndEffectorLimits
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: EndEffectorLimits
    :members:

CartesianPath
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: CartesianPath
    :members:

TaskSpacePlanParameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: TaskSpacePlanParameters
    :members:

State and result data types
------------------------------------------

MoveGripperResult
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: MoveGripperResult
    :members:

WsgResult
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: WsgResult
    :members:

IkResults
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: IkResults
    :members:

SteppedMotionState
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: SteppedMotionState
    :members:

JointValuesCollisions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

How to use it:

Details:

.. autoclass:: JointValuesCollisions
    :members:
