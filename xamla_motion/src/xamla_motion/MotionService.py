from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from future.builtins import *
from future.utils import raise_from, raise_with_traceback

import rospy
from xamlamoveit_msgs.srv import *

from data_types import JointSet
from data_types import MoveGroupDescription


class MotionServices(object):

    def __init__(self):

        self.movej_action_name = 'moveJ_action'
        self.query_move_group_service_name = ('xamlaMoveGroupServices/'
                                              'query_move_group_interface')

    def query_available_move_groups(self):
        for i in range(0, 3):
            try:
                service = rospy.ServiceProxy(
                    self.query_move_group_service_name,
                    QueryMoveGroupInterfaces)
                response = service()
            except rospy.ServiceException as e:
                if i < 2:
                    print ('service call for query available'
                           'move groups failed retry: ')
                else:
                    print ('service call for query available'
                           'move groups failed third time abort ')
                    raise e

        groups = list()
        for g in response.move_group_interfaces:
            if len(g.joint_names) == 0:
                joint_set = JointSet.empty()
            else:
                joint_set = JointSet(g.joint_names)
            groups.append(MoveGroupDescription(g.name, g.sub_move_group_ids,
                                               joint_set, g.end_effector_names,
                                               g.end_effector_link_names))
        return groups
