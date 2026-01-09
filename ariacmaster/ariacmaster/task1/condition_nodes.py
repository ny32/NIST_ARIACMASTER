import py_trees
from py_trees import behaviours
from world_state import world, NORMAL_CELL_VOLTAGE, ALLOWED_VOLTAGE_TOLERANCE

class DefectFound(py_trees.behaviour.Behaviour):
    # Checks if a defect was found in the report
    def __init__(self, name="Defect Found"):
        super().__init__(name)
    def update(self):
        if world.Report.DefectType != "None":
            self.feedback_message = f"Defect Found: {world.Report.DefectType}"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "No Defect Found"
            return py_trees.common.Status.FAILURE

class QueuedCell(py_trees.behaviour.Behaviour):
    def __init__(self, name="Queued Cell"):
        super().__init__(name)
    def update(self):
        if world.cellsQueued > 0:
            self.feedback_message = f"Cells Queued: {world.cellsQueued}"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "No Cells Queued"
            return py_trees.common.Status.FAILURE

class VoltageTester1Free(py_trees.behaviour.Behaviour):
    #Checks if Voltage Tester 1 is free
    def __init__(self, name="Check to see if Voltage tester 1 is Free"):
        super().__init__(name)
    def update(self):
        if world.FreeTesters == "Both" or world.FreeTesters == "Tester 1":
            self.feedback_message = "Voltage Tester 1 is Free"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "Voltage Tester 1 is Busy"
            return py_trees.common.Status.FAILURE
        
class VoltageTester2Free(py_trees.behaviour.Behaviour):
    #Checks if Voltage Tester 1 is free
    def __init__(self, name="Check to see if Voltage tester 2 is Free"):
        super().__init__(name)
    def update(self):
        if world.FreeTesters == "Both" or world.FreeTesters == "Tester 2":
            self.feedback_message = "Voltage Tester 2 is Free"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "Voltage Tester 2 is Busy"
            return py_trees.common.Status.FAILURE
class CurrentCellTolerable(py_trees.behaviour.Behaviour):
    # Check if the Voltage reading of the current cell is within NIST ARIAC tolerances
    def __init__(self, name="Check cell voltage acceptability"):
        super().__init__(name)
    def update(self):
        if abs(world.voltageReading - NORMAL_CELL_VOLTAGE) <= ALLOWED_VOLTAGE_TOLERANCE:
            self.feedback_message = f"Cell voltage {world.voltageReading}V is within acceptable range."
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"Cell voltage {world.voltageReading}V is out of acceptable range."
            return py_trees.common.Status.FAILURE