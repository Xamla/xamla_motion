.. role:: hidden
    :class: hidden-section

Motion client version 2.0
=========================

.. automodule:: xamla_motion
.. currentmodule:: xamla_motion

Overview
------------------------------------------
The main classes to plan and execute trajectories with xamla_motion
are :class:`xamla_motion.v2.MoveGroup` and :class:`xamla_motion.v2.EndEffector`. The difference between both classes 
is the space in which they operate. The :class:`xamla_motion.MoveGroup` class plans and 
executes trajectories where the input is defined in joint space. Whereas
the :class:`xamla_motion.v2.EndEffector` class plans and executes trajectories where the input
is defined in cartesian/task space.

The version 2.0 is an update to the now deprecated implementation :class:`xamla_motion.MoveGroup`
and :class:`xamla_motion.EndEffector`.
In version 2.0 a lot of API changes are made to improve the usability. 

The major change between version 1.0 and 2.0 is that all move methods not directly execute
a trajectory or return a stepped motion client but rather return a move operation. 
This move operations are property containers. It is possible to hand them over and change
the properties if needed. 

With help of the plan method a operation can be transformed to a executable trajectory.  
Here again we get the advantage that it is possible to hand this plan over to other functions.
Furthermore, we now have the freedom to decide on demand if the trajectory should be 
executed supervised or unsupervised.

Example:

.. literalinclude:: ../../examples/example_motion_client_v2.py
    :lines: 21-

MoveGroup
------------------------------------------
.. autoclass:: xamla_motion.v2.MoveGroup
    :members:

EndEffector
------------------------------------------
.. autoclass:: xamla_motion.v2.EndEffector
    :members:

MoveJointsOperation
------------------------------------------
.. autoclass:: xamla_motion.motion_operations.MoveJointsOperation
    :members: 

MoveCartesianOperation
------------------------------------------
.. autoclass:: xamla_motion.motion_operations.MoveCartesianOperation
    :members: 

Plan
------------------------------------------
.. autoclass:: xamla_motion.motion_operations.Plan
    :members: 