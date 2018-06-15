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
            if isinstance(args[0], np.ndarray):
                if (len(args[0].shape) != 2 or
                        args[0].shape[0] != 4 or
                        args[0].shape[1] != 4):
                    raise ValueError('translation_matrix (argument1) '
                                     ' is not a 4x4 numpy array')
                if not issubclass(translation.dtype.type, np.floating):
                    raise TypeError('translation_matrix (argument1) is not a'
                                    'dtype is no floating type')

                self.__translation = args[0][:-1, 3]
                self.__quaternion = Quaternion(matrix=args[0])
        else:
            raise TypeError('translation_matrix (argument1) '
                            'is not of type numpy array')

        # from translation vector and quaternion
        elif len(*args) == 2:
            self._init_with_translation_and_rotation(args[0], args[1])

    def _init_with_translation_and_rotation(self, translation, rotation):
        # translation
        if isinstance(translation, np.ndarray):
            if len(translation.shape) != 1:
                raise ValueError('translation (argument1) is not a one'
                                 ' dimensional numpy array')
            if translation.shape[0] != 3:
                raise ValueError('translation (argument1) numpy array contains'
                                 ' not exactly three values')
            if not issubclass(translation.dtype.type, np.floating):
                raise TypeError('translation dtype is no floating type')

            self.__translation = np.fromiter(translation, float)

        elif ((isinstance(translation, list) or isinstance(translation, tuple))
              and all(isinstance(value, float) for value in translation)):
            if len(translation) != 3:
                raise ValueError('translation list (argument1) contains not '
                                 'exactly three values')

            self.__translation = np.fromiter(translation, float)
        else:
            raise TypeError('translation  (argument1) is not one of the expected'
                            ' types list of float or numpy array of floating')

        # rotation
        if isinstance(rotation, Quaternion):
            self.__quaternion = rotation

        else:
            raise TypeError('rotation (argument2) is not of the expected'
                            ' type Quaternion')
