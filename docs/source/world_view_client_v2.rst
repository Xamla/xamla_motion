.. role:: hidden
    :class: hidden-section

WorldViewClient version 2.0
===========================

.. automodule:: xamla_motion
.. currentmodule:: xamla_motion

Overview
------------------------------------------
One of the main features of ROSVITA is the WorldView.
The WorldView is the place in which the current configration
is visualized. But it not only visualize the current state of
the work cell it is also the place where we can jog the robot,
save specific poses or robot joint configurations and add collision objects
which should be considered when new trajectories are planned and executed.  

.. figure:: ../_static/world_view.jpg
   :scale: 60 %
   :alt: ROSVITA world view

   ROSVITA WorldView example

To get access to the stored entities in WorldView or to add new or manipulate
existing onces the :class:`xamla_motion.v2.WorldViewClient` class can be used.
At the moment following functionalities are provided: 

- Add and remove folders from WorldView 
- Add, get, update, query and remove poses, joint values/configurations, cartesian paths and collision objects. 

The version 2.0 is an update to the now deprecated implementation :class:`xamla_motion.WorldViewClient`.
In version 2.0 a lot of API changes are made to improve the usability. 

Major changes are:

- add and update world view elements is now done with help of the add methods
- update methods are removed
- all methods now use a full path instead of a split path (folder_path and element_name)
- add_folder default behavior not raise a exception if folder already exists
- remove_element default behavior not raise exception if element not exists

Example:

.. literalinclude:: ../../examples/example_world_view_client_v2.py
    :lines: 21-

WorldViewClient
------------------------------------------
.. autoclass:: xamla_motion.v2.WorldViewClient
    :members: