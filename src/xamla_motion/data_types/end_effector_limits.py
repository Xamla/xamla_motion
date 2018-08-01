# end_effector_limits.py
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

import numpy as np


class EndEffectorLimits(object):
    """
    Class which holds hold task space / endeffector limits

    """

    def __init__(self, max_xyz_velocity, max_xyz_acceleration,
                 max_angular_velocity, max_angular_acceleration):
        """
        Initialization of class EndeffectorLimits

        Parameters
        ----------
        max_xyz_velocity : float convertable or None 
            Defines the maximal xyz velocity [m/s]
        max_xyz_acceleration : float convertable 
            Defines the maximal xyz acceleration [m/s^2]
        max_angular_velocity : float convertable or None
            Defines the maximal angular velocity [rad/s]
        max_angular_acceleration : float convertable or None
            Defines the maximal angular acceleration [rad/s^2]

        Returns
        ------
        EndEffectorLimits
            Instance of class EndeffectorLimits

        Raises
        ------
            TypeError : type mismatch
                If one of the parameters is not convertable to float
        """

        self.__max_xyz_velocity = None
        self.__max_xyz_acceleration = None
        self.__max_angular_velocity = None
        self.__max_angular_acceleration = None

        if max_xyz_velocity:
            self.__max_xyz_velocity = float(max_xyz_velocity)
        if max_xyz_acceleration:
            self.__max_xyz_acceleration = float(max_xyz_acceleration)
        if max_angular_velocity:
            self.__max_angular_velocity = float(max_angular_velocity)
        if max_angular_acceleration:
            self.__max_angular_acceleration = float(max_angular_acceleration)

    @property
    def max_xyz_velocity(self):
        """
        max_xyz_velocity : float or None(read only)
            Maximal xyz velocity [m/s]
        """
        return self.__max_xyz_velocity

    @property
    def max_xyz_acceleration(self):
        """
        max_xyz_acceleration : float or None(read only)
            Max xyz acceleration [m/(s^2)]
        """
        return self.__max_xyz_acceleration

    @property
    def max_angular_velocity(self):
        """
        max_angular_velocity: float or None(read only)
            Max angular velocity[rad/s]
        """
        return self.__max_angular_velocity

    @property
    def max_angular_acceleration(self):
        """
        max_angular_acceleration: float or None(read only)
            Max angular acceleration[rad/(s ^ 2)]
        """
        return self.__max_angular_acceleration

    def __str__(self):
        s = '\n'.join([k+' = ' + str(v) for k, v in self.__dict__.items()])
        s = s.replace('_'+self.__class__.__name__+'__', '')
        return 'EndeffectorLimits\n'+s

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        r_tol = 1.0e-13
        a_tol = 1.0e-14

        if not isinstance(other, self.__class__):
            return False

        if id(other) == id(self):
            return True

        for k, v in self.__dict__:
            if not np.isclose(v,
                              other.__dict__[k],
                              rtol=r_tol, atol=a_tol):
                return False

        return True

    def __ne__(self, other):
        return not self.__eq__(other)
