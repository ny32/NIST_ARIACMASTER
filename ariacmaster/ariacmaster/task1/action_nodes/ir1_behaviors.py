import py_trees
from py_trees import behaviours
import time
import random

from py_trees.common import Status
from world_state import world, Report

class IR1MoveToPickup(py_trees.behaviour.Behaviour):
    def __init__(self, name="IR1MoveToPickup"):
        super().__init__(name)

    def update(self):
        if world.IR1Location != "Conveyor":
            world.IR1Free = False
            self.feedback_message = "IR1 moving to Conveyor"
            time.sleep(2)  # Simulate movement time
            world.IR1Location = "Conveyor"
            world.IR1Free = True
            self.feedback_message = "IR1 reached Conveyor"
        return py_trees.common.Status.FAILURE
    
class IR1GraspCell(py_trees.behaviour.Behaviour):
    def __init__(self, name="IR1GraspCell"):
        super().__init__(name)

    def update(self):
        if world.IR1Free and world.IR1Location == "Conveyor":
            world.IR1Free = False
            world.cellsQueued -= 1
            self.feedback_message = "IR1 grasping cell"
            time.sleep(0.5)  # Simulate grasping time
            world.IR1Grasping = True
            self.feedback_message = "IR1 grasped cell"
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class IR1MoveToTester1(py_trees.behaviour.Behaviour):
    def __init__(self, name="Move IR1 to Tester 1"):
        super().__init__(name)
    def update(self):
        if world.IR1Grasping and world.IR1Location != "Tester 1":
            self.feedback_message = "IR1 moving to Tester 1"
            time.sleep(2)  # Simulate movement time
            world.IR1Location = "Tester 1"
            self.feedback_message = "IR1 reached Tester 1"
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class IR1MoveToTester2(py_trees.behaviour.Behaviour):
    def __init__(self, name="Move IR1 to Tester 2"):
        super().__init__(name)
    def update(self):
        if world.IR1Grasping and world.IR1Location != "Tester 2":
            self.feedback_message = "IR1 moving to Tester 2"
            time.sleep(2)  # Simulate movement time
            world.IR1Location = "Tester 2"
            self.feedback_message = "IR1 reached Tester 2"
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class IR1PlaceInTester(py_trees.behaviour.Behaviour):
    def __init__(self, name="IR1GraspCell"):
        super().__init__(name)

    def update(self):
        if world.IR1Grasping and (world.IR1Location == "Tester 1" or world.IR1Location == "Tester 2"):
            self.feedback_message = f"IR1 placing cell in {world.IR1Location}"
            time.sleep(0.5)  # Simulate placing time
            world.IR1Grasping = False
            world.IR1Free = True
            self.feedback_message = f"IR1 placed cell in {world.IR1Location}"
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE