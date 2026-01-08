import py_trees
from py_trees import behaviours
from world_state import world, Report

import time
import random

class IR1MoveToPickup(py_trees.behaviour.Behaviour):
    def __init__(self, name="IR1MoveToPickup"):
        super(IR1MoveToPickup, self).__init__(name)

    def update(self):
        # ...
        return py_trees.common.Status.FAILURE