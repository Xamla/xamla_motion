.. role:: hidden
    :class: hidden-section

Motion client
=========================

.. automodule:: xamla_motion
.. currentmodule:: xamla_motion

Overview
------------------------------------------
The main classes to plan and execute trajectories with xamla_motion
are :class:`xamla_motion.MoveGroup` and :class:`xamla_motion.EndEffector`. The difference between both classes 
is the space in which they operate. The :class:`xamla_motion.MoveGroup` class plans and 
executes trajectories where the input is defined in joint space. Whereas
the :class:`xamla_motion.EndEffector` class plans and executes trajectories where the input
is defined in cartesian/task space. 

Example:

.. literalinclude:: ../../examples/example_motion_client.py
    :lines: 21-

MoveGroup
------------------------------------------
.. autoclass:: MoveGroup
    :members:

EndEffector
------------------------------------------
.. autoclass:: EndEffector
    :members:
