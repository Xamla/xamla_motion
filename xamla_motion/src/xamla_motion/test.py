from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from future.builtins import *
from future.utils import raise_from, raise_with_traceback

from MotionService import MotionServices

import pdb

motion_service = MotionServices()

groups = motion_service.query_available_move_groups()

for group in groups:
    print(group)

pdb.set_trace()
