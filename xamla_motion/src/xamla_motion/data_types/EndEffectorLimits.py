from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from future.builtins import *
from future.utils import raise_from, raise_with_traceback


class EndEffectorLimits(object):
    """
    Class which holds hold task space / endeffector limits

    Attributes
    ----------
    max_xyz_velocity : float (read only)
        Maximal xyz velocity [m/s]
    max_xyz_acceleration : float (read only)
        Max xyz acceleration [m/(s^2)]
    max_angular_velocity : float (read only)
        Max angular velocity [rad/s]
    max_angular_acceleration : float (read only)
        Max angular acceleration [rad/(s^2)]
    """

    def __init__(self, max_xyz_velocity, max_xyz_acceleration,
                 max_angular_velocity, max_angular_acceleration):
        """
        Initialization of class EndeffectorLimits

        Parameters
        ----------
        max_xyz_velocity : float convertable
            Defines the maximal xyz velocity [m/s]
        max_xyz_acceleration : float convertable (read only)
            Defines the maximal xyz acceleration [m/s^2]
        max_angular_velocity : float convertable (read only)
            Defines the maximal angular velocity [rad/s]
        max_angular_acceleration : float convertable (read only)
            Defines the maximal angular acceleration [rad/s^2]

        Yields
        ------
            Instance of class EndeffectorLimits

        Raise
        -----
            TypeError : type mismatch
                If one of the parameters is not convertable to float
        """
        self.__max_xyz_velocity = float(max_xyz_velocity)
        self.__max_xyz_acceleration = float(max_xyz_acceleration)
        self.__max_angular_velocity = float(max_angular_velocity)
        self.__max_angular_acceleration = float(max_angular_acceleration)

    @property
    def max_xyz_velocity(self):
        """max_xyz_velocity read only"""
        return self.__max_xyz_velocity

    @property
    def max_xyz_acceleration(self):
        """max_xyz_acceleration read only"""
        return self.__max_xyz_acceleration

    @property
    def max_angular_velocity(self):
        """max_angular_velocity read only"""
        return self.__max_angular_velocity

    @property
    def max_angular_acceleration(self):
        """max_angular_acceleration read only"""
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
