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

class setIR2TargetCellTo1(py_trees.behaviour.Behaviour):
    def __init__(self, name="Set IR2 Target Cell to Cell 1"):
        super().__init__(name)

    def update(self):
        if world.IR2TargetCell == "None":
            world.IR2TargetCell = "Cell 1"
            self.feedback_message = "Set IR2 to target Cell 1"
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
    
class setIR2TargetCellTo2(py_trees.behaviour.Behaviour):
    def __init__(self, name="Set IR2 Target Cell to Cell 1"):
        super().__init__(name)

    def update(self):
        if world.IR2TargetCell == "None":
            world.IR2TargetCell = "Cell 2"
            self.feedback_message = "Set IR2 to target Cell 2"
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class TakeCurrentCellVoltage(py_trees.behaviour.Behaviour):
    # Takes Voltage reading of the cell in the tester
    def __init__ (self, name="Take Voltage Reading"):
        super().__init__(name)
    def update(self):
        if world.IR2TargetCell != "None":
            time.sleep(1) # Time delay for taking voltage reading
            self.feedback_message = f"Voltage reading taken for {world.IR2TargetCell}"
            world.voltageReading = ( random.randint(124, 129) ) / 10 
            """
            Possible Values: 12.4, 12.5, 12.6, 12.7, 12.8, 12.9
            Not Allowed: 12.4V (16.7% probability of occurence)
            """
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
    
class IR2MoveToCurrentCell(py_trees.behaviour.Behaviour):
    def __init__(self, name="Move IR1 to Current Cell"):
        super().__init__(name)
    def update(self):
        if world.IR2Free:
            world.IR2Free = False
            if world.IR2TargetCell == "Cell 1":
                self.feedback_message = "IR2 moving to Tester 1"
                time.sleep(2)  # Simulate movement time
                world.IR2Location = "Tester 1"
                self.feedback_message = "IR2 reached Tester 1"
            elif world.IR2TargetCell == "Cell 2":
                self.feedback_message = "IR2 moving to Tester 2"
                time.sleep(2)  # Simulate movement time
                world.IR2Location = "Tester 2"
                self.feedback_message = "IR2 reached Tester 2"
            world.IR2Free = True
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class IR1GraspCell(py_trees.behaviour.Behaviour):
    def __init__(self, name="IR2 Grasp Cell"):
        super().__init__(name)

    def update(self):
        if world.IR2Free and (world.IR2Location == "Tester 1" or world.IR2Location == "Tester 2"):
            world.IR2Free = False
            self.feedback_message = "IR2 grasping cell"
            time.sleep(0.5)  # Simulate grasping time
            world.IR2Grasping = True
            self.feedback_message = "IR1 grasped cell"
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class FindNearestFreeAGVSlot(py_trees.behaviour.Behaviour):
    def __init__(self, name="Locate Nearest Cell"):
        super().__init__(name)
    
    def update(self):
        for AGV in range(0,3):
            for Slot in range(0, 4):
                if world.AGVSlots[AGV][Slot] != "X":
                    world.FreeAGVSlot = [AGV, Slot]
                    self.feedback_message = f"Found available slot {Slot} on AGV {AGV}."
                    return py_trees.common.Status.SUCCESS
        self.feedback_message = "No available AGV slots at this time."
        return py_trees.common.Status.FAILURE

class IR2MoveToAGVSlot(py_trees.behaviour.Behaviour):
    def __init__(self, name="Move IR2 to free AGV Slot"):
        super().__init__(name)
    def update(self):
        if world.IR2Grasping and (world.IR2Location == "Tester 1" or world.IR2Location == "Tester 2"):
            self.feedback_message = f"IR2 moving to AGV {world.AGVSlots[0]}, targeting slot #{world.AGVSlots[1]}"
            time.sleep(2)  # Simulate movement time
            if world.AGVSlots == 1:
                self.feedback_message = "IR2 reached AGV 1"
            elif world.AGVSlots == 2:
                self.feedback_message = "IR2 reached AGV 2"
            else: #Logically, 3 is the only one left
                self.feedback_message = "IR2 reached AGV 3"                
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
    
class IR2MoveToAGVSlot(py_trees.behaviour.Behaviour):
    def __init__(self, name="Move IR2 to free AGV Slot"):
        super().__init__(name)
    def update(self):
        if world.IR2Grasping and (world.IR2Location == "Tester 1" or world.IR2Location == "Tester 2"):
            self.feedback_message = f"IR2 moving to AGV {world.AGVSlots[0]}, targeting slot #{world.AGVSlots[1]}"
            time.sleep(2)  # Simulate movement time
            if world.AGVSlots == 1:
                self.feedback_message = "IR2 reached AGV 1"
            elif world.AGVSlots == 2:
                self.feedback_message = "IR2 reached AGV 2"
            else: #Logically, 3 is the only one left
                self.feedback_message = "IR2 reached AGV 3"                
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class IR2PlaceInSlot(py_trees.behaviour.Behaviour):
    def __init__(self, name="IR2 places cell into slot"):
        super().__init__(name)
    def update(self):
        if world.IR2Grasping and (world.IR2Location == "AGV 1" or world.IR2Location == "AGV 2" or world.IR2Location == "AGV 3"):
            time.sleep(0.5) # Simulate placing into slot
            self.feedback_message = f"Placing into AGV {world.AGVSlots[0]}, Slot {world.AGVSlots[1]}"
            world.IR2Grasping = False # Releases Cell
            world.IR2Free = True #IR2 is done with step
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

# -------------------------------------------------------------------------------------
# BE CAUTIOUS: Make sure arm returns back or code is able to identify if it hasn't!!!!!
# -------------------------------------------------------------------------------------

class IR2MoveToRecycling(py_trees.behaviour.Behaviour):
    def __init__(self, name="IR2 moving to recycling"):
        super().__init__(name)
    def update(self):
        if world.IR2Grasping and world.IR2Location != "Recycling Bin":
            self.feedback_message = "Moving over to recycling"
            time.sleep(2) # Simulate movement time
            world.IR2Location = "Recycling Bin"
            self.feedback_message = "IR2 reached the recycling bin"
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class IR2ReleaseCell(py_trees.behaviour.Behaviour):
    def __init__(self, name="IR2 releases the cell to drop it"):
        super().__init__(name)
    def update(self):
        if world.IR2Grasping:
            self.feedback_message = "IR2 is releasing the cell"
            time.sleep(0.5) # Simulate release time
            world.IR2Grasping = False
            world.IR2Free = True
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE