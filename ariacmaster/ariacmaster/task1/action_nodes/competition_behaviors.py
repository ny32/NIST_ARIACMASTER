import py_trees
from py_trees import behaviours
import time

from py_trees.common import Status
from world_state import world, Report

class StartCompetition(py_trees.behaviour.Behaviour):
    # Starts the competition
    def __init__(self, name="Start Competition"):
        super().__init__(name)
    def update(self):
        self.feedback_message = "Starting competition..."
        time.sleep(5) # Simulate delay for competition start
        self.feedback_message = "Competition started"
        return py_trees.common.Status.SUCCESS
    
class EndCompetition(py_trees.behaviour.Behaviour):
    # Ends the competition
    def __init__(self, name="End Competition"):
        super().__init__(name)
    def update(self):
        self.feedback_message = "Ending competition..."
        time.sleep(3) # Simulate delay for competition end
        self.feedback_message = "Competition ended"
        return py_trees.common.Status.SUCCESS