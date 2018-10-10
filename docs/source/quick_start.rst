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

First steps with ROSVITA and xamla_motion
----------------------------------------------------------

Due to the fact that xamla_motion is a client library which interacts with `ROSVITA <https://xamla.com/en/#about>`_ we first have to
do some setup work in `ROSVITA <https://xamla.com/en/#about>`_ itself, before we can explore the capabilities of xamla_motion. 

Please first start a `ROSVITA <https://xamla.com/en/#about>`_ instance on your local machine. A 'how to' is available in 
the `ROSVITA Documentation <http://docs.xamla.com/rosvita/Getting_Started.html>`_.

After ROSVITA is started, please login and create a new project. A 'how to' for this steps is also available:

-  `login <http://docs.xamla.com/rosvita/Main_View.html>`_.
-  `ROSVITA Documentation <http://docs.xamla.com/rosvita/New_Project.html>`_.

Then add a ur5 and a WSG-25 gripper as simulated components to the configuration, compile and start ROS (`how to <http://docs.xamla.com/rosvita/Getting_Started.html>`_)