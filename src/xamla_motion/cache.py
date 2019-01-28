import os
import io
import gzip
import pickle
import gc
from functools import wraps
from typing import Callable
from .xamla_motion_exceptions import (ServiceException,
                                      XamlaMotionException)
# TODO: IS the timer necessary here?
# from laundrometer_python.utility import Timer


class Cache(object):
    def __init__(self, id, directory_path='./python_trajectory_cache'):
        self.directory_path = directory_path or './python_trajectory_cache'
        assert(id is not None), "id should have a str value"
        self.id = id
        self.data = {}
        self.tmp_data = {}

    def load(self):
        file_name = self.id + '.pickle'
        cached_file = os.path.join(self.directory_path, file_name)
        if os.path.isfile(cached_file):
            with gzip.open(cached_file, 'rb') as f:
                data = pickle.load(io.BufferedReader(f))
                self.data = data

    def dump(self):
        #with Timer('TrajetoryCache:dump'):
        file_name = self.id + '.pickle'
        if not os.path.isdir(self.directory_path):
            os.makedirs(self.directory_path, exist_ok=True)

        cached_file = os.path.join(self.directory_path, file_name)
        gc.disable()
        with gzip.open(cached_file, 'wb') as f:
            pickle.dump(self.data, io.BufferedWriter(f), protocol=pickle.HIGHEST_PROTOCOL)
        gc.enable()

    def add(self, key, payload, persistent=True):
        if persistent:
            self.data[key] = payload
        else:
            self.tmp_data[key] = payload

    def get(self, key):
        result = None
        if key in self.data:
            result = self.data[key]
        elif key in self.tmp_data:
            result = self.tmp_data[key]
        else:
            print('[{}.get] Key "{}" is not yet in database'.format(
                self.__class__.__name__, key))

        return result

    def remove(self, key):
        if key in self.data:
            del self.data[key]
        elif key in self.tmp_data:
            del self.tmp_data[key]
        else:
            print('[{}.remove] Key "{}" is not yet in database'.format(
                self.__class__.__name__, key))

    def clear(self):
        self.data = {}
        self.dump()
