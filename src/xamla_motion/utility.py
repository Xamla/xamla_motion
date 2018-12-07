# utility.py
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

import asyncio
import functools
import re
import signal
from collections import Iterable
from typing import List

import matplotlib.cm as cmx
import matplotlib.colors as colors
import matplotlib.pyplot as plt
import rospy
from xamlamoveit_msgs.srv import QueryLock, QueryLockRequest

from .data_types import JointSet, JointTrajectory, JointTrajectoryPoint
from .xamla_motion_exceptions import ServiceException

resource_lock_srv_name = '/xamlaResourceLockService/query_resource_lock'


class ResourceLock(object):

    def __init__(self, success: bool, resource_ids: List[str], lock_id: str,
                 creation_date: str, expiration_date: str):

        self.__success = success
        self.__resource_ids = resource_ids
        self.__lock_id = lock_id
        self.__creation_date = creation_date
        self.__expiration_date = expiration_date

    @property
    def success(self):
        """
        success : bool
            If true locking was successful
        """
        return self.__success

    @property
    def resource_ids(self):
        """
        resource_ids : str
            ID of lock resources necessary to release them later
        """
        return self.__resource_ids

    @property
    def lock_id(self):
        """
        lock_id : str
            UUID which identifies the client who request the lock
        """
        return self.__lock_id

    @property
    def creation_date(self):
        """
        creation_date : str
            Date of lock creation
        """
        return self.__creation_date

    @property
    def expiration_date(self):
        """
        expiration_date : str
            Date of lock expiration
        """
        return self.expiration_date


class LeaseBaseLock(object):
    """
    Class to lock resource

    Parameters
    ----------
    resource_ids : Iterable[str]
        Ids of the resources where lock them is requested

    Returns
    -------
    LeaseBaseLock
        Instance of LeaseBaseLock

    Raises
    ------
    TypeError
        If resource_ids is not Iterable of str
    ServiceError
        If lock service is no available or finish unsuccessfully

    Examples
    --------
    The lock and release can be implemented comfortably with
    python's with statement
    >>> with LeaseBaseLock(resource_ids) as lock_handle:
    >>>     do something with resources
    """

    def __init__(self, resource_ids: List[str], lock_id: str=''):

        self.__request = QueryLockRequest()
        self.__request.id_resources = list(resource_ids)
        self.__request.id_lock = str(lock_id)

        self.__lock_service = rospy.ServiceProxy(resource_lock_srv_name,
                                                 QueryLock)

        self.__resource_lock = None

    def __enter__(self):
        self._call_lock_service(release=False)
        return self.__resource_lock

    def __exit__(self, *exc):
        self._call_lock_service(release=True)

    @property
    def resource_lock(self):
        """
        resource_lock: ResourceLock or None
            Currently locked resources
        """
        return self.__resource_lock

    def _call_lock_service(self, release=False):
        self.__request.release = release
        if release:
            self.__request.id_lock = self.__resource_lock.lock_id

        try:
            response = self.__lock_service.call(self.__request)
        except rospy.ServiceException as exc:
            raise ServiceException('lock service is not available') from exc

        if not response.success:
            raise ServiceException('lock service finish not succuessfully'
                                   ', reason: {}'.format(response.error_msg))

        if release:
            self.__resource_lock = None
        else:
            self.__resource_lock = ResourceLock(response.success, response.id_resources,
                                                response.id_lock, response.creation_date,
                                                response.expiration_date)


class ROSNodeSteward(object):
    """
    Maintain ros node

    If no rosnode exist create a new rosnode
    """

    def __init__(self):
        if (re.sub('[^A-Za-z0-9]+', '', rospy.get_name()) == 'unnamed'):
            rospy.init_node('xamla_motion', anonymous=True)


def register_asyncio_shutdown_handler(asyncio_loop):
    """
    Register signals SIGTERM and SIGINT at asyncio loop

    If one of both signals is raised shutdown asyncio loop
    properly

    Parameters
    ----------
    asyncio_loop
        asyncio event loop on which the signals should be registered
    """

    def _shutdown(loop, reason):
        print('shutdown asyncio due to : {}'.format(reason), flush=True)
        tasks = asyncio.gather(*asyncio.Task.all_tasks(loop=loop),
                               loop=loop, return_exceptions=True)
        tasks.add_done_callback(lambda t: loop.stop())
        tasks.cancel()

        # Keep the event loop running until it is either destroyed or all
        # tasks have really terminated
        while not tasks.done() and not loop.is_closed():
            loop.run_forever()

    asyncio_loop.add_signal_handler(signal.SIGTERM,
                                    functools.partial(_shutdown,
                                                      asyncio_loop,
                                                      signal.SIGTERM))
    asyncio_loop.add_signal_handler(signal.SIGINT,
                                    functools.partial(_shutdown,
                                                      asyncio_loop,
                                                      signal.SIGINT))


def plot_joint_trajectory(trajectory: JointTrajectory, title: str):
    joint_names = trajectory.joint_set.names
    jet = plt.get_cmap('jet')
    cNorm = colors.Normalize(vmin=0, vmax=len(joint_names))
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)

    time_from_start = [t.total_seconds() for t in trajectory.time_from_start]

    def plot(ax, x, values, labels, title, ylabel):
        plt_lines = []
        for i, label in enumerate(labels):
            y = [v[i] for v in values]
            plt_line, = ax.plot(x,
                                y,
                                lw=1.5,
                                color=scalarMap.to_rgba(i),
                                label=label)
            plt_lines.append(plt_line)
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.set_xlabel('time from start [s]')
        return plt_lines

    plots = 1
    if trajectory.has_velocity:
        plots += 1
    if trajectory.has_acceleration:
        plots += 1
    if trajectory.has_effort:
        plots += 1

    plt_lines = []
    fig, ax = plt.subplots(plots, 1)
    fig.suptitle(title, fontsize=14)
    plt_lines.append(plot(ax[0], time_from_start,
                          trajectory.positions,
                          joint_names,
                          'joint positions',
                          'joint positions [rad]'))

    if trajectory.has_velocity:
        plt_lines.append(plot(ax[1], time_from_start,
                              trajectory.velocities,
                              joint_names,
                              'joint velocities',
                              'joint velocities [rad/s]'))

    if trajectory.has_acceleration:
        plt_lines.append(plot(ax[2], time_from_start,
                              trajectory.accelerations,
                              joint_names,
                              'joint accelerations',
                              'joint accelerations [rad/s^2]'))

    if trajectory.has_effort:
        plt_lines.append(plot(ax[3], time_from_start,
                              trajectory.efforts,
                              joint_names,
                              'joint efforts',
                              'joint efforts [N]'))

    legend = fig.legend(plt_lines[0], joint_names,
                        loc='center right', fancybox=True, shadow=True)

    lined = {}
    for i, leg_line in enumerate(legend.get_lines()):
        leg_line.set_picker(5)
        lined[leg_line] = [l[i] for l in plt_lines]

    def onpick(event):
        legline = event.artist
        origlines = lined[legline]
        for origline in origlines:
            vis = not origline.get_visible()
            origline.set_visible(vis)
            if vis:
                legline.set_alpha(1.0)
            else:
                legline.set_alpha(0.2)
        fig.canvas.draw()

    fig.canvas.mpl_connect('pick_event', onpick)

    plt.show()
