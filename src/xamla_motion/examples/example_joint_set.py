# example_joint_set.py
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

from xamla_motion.data_types import JointSet


def main():
    # create a empty instance of JointSet
    empty = JointSet.empty()
    print('empty joint set:', empty)

    # create a joint set with two joints by string
    joint_set1 = JointSet('Joint1, Joint2')
    print('joint set 1:', joint_set1)

    # create a joint set with three joints by iterable (here list)
    joint_set2 = JointSet(['Joint1', 'Joint2', 'Joint3'])
    print('joint set 2:', joint_set2)

    # get joint names joint set 1 contains
    print('get list of names in joint set 1', joint_set1.names)

    # test if joint set 1 is subset of joint set 2
    is_subset = joint_set1.is_subset(joint_set2)
    print('is joint set 1 subset of joint set 2', is_subset)

    # test if joint set 2 is superset of joint set 1 by greater operator
    is_superset = joint_set2 > joint_set1
    print('is joint set 2 superset of joint set 1 by operator', is_superset)

    # create JointSet which is the union of joint_set2 and another jointset
    joint_set3 = joint_set1.union(JointSet('Joint3, Joint4'))
    print('joint set 3:', joint_set3)


if __name__ == '__main__':
    main()
