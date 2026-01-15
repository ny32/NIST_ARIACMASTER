import py_trees
from py_trees import behaviours
import time
import random

from py_trees.common import Status
from world_state import world, Report

class ConstructLIDARModel(py_trees.behaviour.Behaviour):
    # Scans the cell using LIDAR and constructs a 3D model
    def __init__(self, name="Construct LIDAR Model"):
        super().__init__(name)

    def update(self):
        time.sleep(1) # Time delay for cell arrival
        self.feedback_message = "Cell Detected. Constructing model..."
        time.sleep(0.5)
        self.feedback_message = "LIDAR Model Constructed"
        return py_trees.common.Status.SUCCESS

class PopulateReport(py_trees.behaviour.Behaviour):
    # Populates inspection report based on LIDAR model
    def __init__(self, name="Populate Report"):
        super().__init__(name)
    
    def update(self):
        # Simulate randomness in defect detection
        randomValue = random.randint(0, 10)
        world.Report = Report(
            Status=False,
            DefectType="Dent" if randomValue == 10 
                        else "Scratch" if randomValue == 9
                        else "Bulge" if randomValue == 8 
                        else "None"
        )
        self.feedback_message = (f"Report was populated with {world.Report.DefectType + "detected." if world.Report.DefectType != 'None' else "nothing detected."}")
        return py_trees.common.Status.SUCCESS
    
class SubmitReport(py_trees.behaviour.Behaviour):
    # Submits inspection report
    def __init__(self, name="Submit Report"):
        super().__init__(name)
    def update(self):
        self.feedback_message = "Report Submitted."
        return py_trees.common.Status.SUCCESS

class OpenInspectionDoor(py_trees.behaviour.Behaviour):
    # Opens the inspection door
    def __init__(self, name="Open Inspection Door"):
        super().__init__(name)
    def update(self):
        world.InspectionDoor = "Open"
        self.feedback_message = "Inspection Door Opened."
        world.cellsQueued += 1
        return py_trees.common.Status.SUCCESS