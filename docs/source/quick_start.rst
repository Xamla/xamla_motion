Quick start guide
=================

What is xamla_motion?
----------------------------------------------------------

xamla_motion is a python client library to interact with `ROSVITA <https://xamla.com/en/#about>`_ based on our server 
implementation `xamlamoveit  <https://github.com/Xamla/xamlamoveit>`_. With help of xamla_motion it is possible to directly
plan and execute trajetories on simulated or real robots, control gripper or manipulate ROSVITA's WorldView from python 3.5 or above.
Further more xamla_motion defines some standard data types wish are usefull in robotic applications like poses or trajectories.

How to install?
----------------------------------------------------------

xamla_motion is already preinstalled in `ROSVITA <https://xamla.com/en/#about>`_. Therefore, no additional installation is necessary.
If you want to use xamla_motion not in the ROSVITA docker image but from your own machine, please clone the 
'xamla_motion  repository <https://github.com/Xamla/xamla_motion>' into your local ROS catkin repository and build it. Please, make sure
that all dependencies in setup.py and package.xml are met.

First steps with ROSVITA and xamla_motion by example
----------------------------------------------------------

Create a ROSVITA project
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Due to the fact that xamla_motion is a client library which interacts with `ROSVITA <https://xamla.com/en/#about>`_ we first have to
do some setup work in `ROSVITA <https://xamla.com/en/#about>`_ itself, before we can explore the capabilities of xamla_motion. 

Please first start a `ROSVITA <https://xamla.com/en/#about>`_ instance on your local machine. A 'how to' is available in 
the `ROSVITA Documentation <http://docs.xamla.com/rosvita/Getting_Started.html>`_.

After ROSVITA is started, please login and create a new project. A 'how to' for this steps is also available:

-  `login <http://docs.xamla.com/rosvita/Main_View.html>`_
-  `create new project <http://docs.xamla.com/rosvita/New_Project.html>`_

Then add a ur5 and a WSG-25 gripper as simulated components to the configuration, compile, start ROS and switch to WorldView 
(`how to <http://docs.xamla.com/rosvita/Getting_Started.html>`_).


Getting to know xamla_motion data types
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

xamla_motion data types are following two design decisions. The first decision 
is that the data types are immutable. Therefore, it is not possible to change a
value of a existing data type instance but rather it is neccessary to create
a new instance which contains the changes.

The second design decision is that all types heavily depend on numpy and especially 
represent all tensors with help of numpy ndarrays.

When people start to think of how to solve a specific robotics problem many will start
to define their problem in poses the robot has to reach and do specific actions with
their tools. Therefore, one of the most important data types xamla_motion defines is
the Pose data type. 

A pose in xamla_motion is defined as three dimensional translation vector [meter], a quaternion
which represents the orientation. For quaternion representation xamla_motion use 
`PyQuaternion <https://kieranwynn.github.io/pyquaternion/>`_. Therefore, the creation of
an instance of xamla_motions Pose data type is possible with this to parameters.

.. code::

    >>> from xamla_motion.data_types import Pose
    >>> import numpy as np
    >>> from pyquaternion import Quaternion
    >>> p = [0.502, 0.258, 0.367]
    >>> q = Quaternion(w=0.304, x=0.527, y=0.687, z=0.396)
    >>> pose = Pose(p, q)
    >>> print(pose)
    Pose:
    translation.x : 0.502
    translation.y : 0.258
    translation.z : 0.367
    quaternion.w : 0.304
    quaternion.x : 0.527
    quaternion.y : 0.687
    quaternion.z : 0.396
    frame_id : world
    >>> pose.translation
    array([ 0.502,  0.258,  0.367])
    >>> pose.quaternion
    Quaternion(0.304, 0.527, 0.687, 0.396)

As you can see the pose data type also has a third parameter frame id. This parameter specifics
in which coordinate system the pose is defined. The default value is world.

Another common representation of poses in robotics is the representation as a 4x4 matrix
in homogenous coordinates. The xamla_motion pose data type can also be created from this
representation or vise versa.

.. code::

    >>> from xamla_motion.data_types import Pose
    >>> import numpy as np
    >>> pose = Pose.from_transformation_matrix(np.eye(4))
    >>> print(pose)
    Pose:
    translation.x : 0.0
    translation.y : 0.0
    translation.z : 0.0
    quaternion.w : 1.0
    quaternion.x : 0.0
    quaternion.y : 0.0
    quaternion.z : 0.0
    frame_id : world
    >>> pose.transformation_matrix()
    array([[ 1.,  0.,  0.,  0.],
       [ 0.,  1.,  0.,  0.],
       [ 0.,  0.,  1.,  0.],
       [ 0.,  0.,  0.,  1.]])

Now we have poses which are represented as xamla_motion pose data types. But why we should
represent poses with help of this data types. The answer is, because we will do less errors
with help of them. For example we can do simple transformations like translate 0.5 meter in 
+x and +y direction.

.. code::

    >>> from xamla_motion.data_types import Pose
    >>> import numpy as np
    >>> translation = np.asarray([0.5, 0.5, 0.0])
    >>> pose = Pose.from_transformation_matrix(np.eye(4))
    >>> print(pose)
    Pose:
    translation.x : 0.0
    translation.y : 0.0
    translation.z : 0.0
    quaternion.w : 1.0
    quaternion.x : 0.0
    quaternion.y : 0.0
    quaternion.z : 0.0
    frame_id : world
    >>> pose.translate(translation)
    Pose:
    translation.x : 0.5
    translation.y : 0.5
    translation.z : 0.0
    quaternion.w : 1.0
    quaternion.x : 0.0
    quaternion.y : 0.0
    quaternion.z : 0.0
    frame_id : world 


In the future more information about the main data types will be added.
But for know take a look into the other chapters to learn following:

- how to move a robot with help of the :doc:`motion client classes <motion_client>`
- how to control a gripper with help of the :doc:`gripper client classes <gripper_client>`
- how to interact with ROSVITA WorldView with help of the :doc:`world view client <world_view_client>`
- more details about the xamla_motion :doc:`data types <data_types>`

