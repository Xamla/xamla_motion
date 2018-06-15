#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from future.builtins import *
from functools import total_ordering

from pyquanternion import Quaternion
import numpy as np


class Pose(object):
    """
    Pose defined by three dimensional translation and rotation

    The translation is representated by a three dimensional 
    column vector. The rotation is represented by a 
    quaternion. The pose itself is idenified by the frame_id
    attribute

    """

    def __init__(self, *args, **kwargs):
        """

        """
        # from transformation matrix
        if len(*args) == 1:
            pass

        # from translation vector and quaternion
        elif len(*args) == 2:
            self._init_with_translation_and_rotation(args[0], args[1])

    def _init_with_translation_and_rotation(self, translation, rotation):
        # translation
        if isinstance(translation, np.ndarray):
            if len(translation.shape) != 1:
                raise ValueError('translation is not a one'
                                 ' dimensional numpy array')
            if translation.shape[0] != 3:
                raise ValueError('translation numpy array contains'
                                 ' not exactly three values')
            if not issubclass(translation.dtype.type, np.floating):
                raise TypeError('translation dtype is no floating type')

            self.__translation = np.fromiter(translation, float)

        elif ((isinstance(translation, list) or isinstance(translation, tuple))
              and all(isinstance(value, float) for value in translation)):
            if len(translation) != 3:
                raise ValueError('translation list contains not '
                                 'exactly three values')

            self.__translation = np.fromiter(translation, float)
        else:
            raise TypeError('translation is not one of the expected'
                            ' types list of float or numpy array of floating')

        # rotation
        if isinstance(rotation, Quaternion):
            self.__quaternion = rotation

        else:
            raise TypeError('rotation is not of the expected'
                            ' type Quaternion')
