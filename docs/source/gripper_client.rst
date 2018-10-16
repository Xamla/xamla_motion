.. role:: hidden
    :class: hidden-section

Gripper client
=========================

.. automodule:: xamla_motion
.. currentmodule:: xamla_motion

Overview
------------------------------------------
As the motion client classes represent the interface to plan and
execute trajectories the gripper client classes are the interface
to control different kinds of gripper. At the moment two kinds are
support. The classes :class:`xamla_motion.CommonGripperProperties` and :class:`xamla_motion.CommonGripper` 
represents a common inferface for grippers. Due to this generalization
the classes only provide methods to control a gripper in a way that many
gripper have in common. The classes :class:`xamla_motion.WeissWsgGripperProperties` and :class:`xamla_motion.WeissWsgGripper`
represent a specialized interface for the capabilities of a smart gripper like
the Weiss WSG series offers.

Example:

.. literalinclude:: ../../examples/example_gripper_client.py
    :lines: 21-

CommonGripper
------------------------------------------
.. autoclass:: CommonGripperProperties
    :members:

.. autoclass:: CommonGripper
    :members:

WeissWsgGripper
------------------------------------------
.. autoclass:: WeissWsgGripperProperties
    :members:

.. autoclass:: WeissWsgGripper
    :members:
