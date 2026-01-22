import py_trees
from world.main import WORLD
from core.constants import NORMAL_CELL_VOLTAGE, ALLOWED_VOLTAGE_TOLERANCE

class DefectFound(py_trees.behaviour.Behaviour):
    # Checks if a defect was found in the report
    def __init__(self, name="Defect Found"):
        super().__init__(name)
    def update(self):
        if WORLD.Report.DefectType != "None":
            self.feedback_message = f"Defect Found: {WORLD.Report.DefectType}"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "No Defect Found"
            return py_trees.common.Status.FAILURE

class QueuedCell(py_trees.behaviour.Behaviour):
    def __init__(self, name="Queued Cell"):
        super().__init__(name)
    def update(self):
        if WORLD.cellsQueued > 0:
            self.feedback_message = f"Cells Queued: {WORLD.cellsQueued}"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "No Cells Queued"
            return py_trees.common.Status.FAILURE

class VoltageTester1Free(py_trees.behaviour.Behaviour):
    #Checks if Voltage Tester 1 is free
    def __init__(self, name="Check to see if Voltage tester 1 is Free"):
        super().__init__(name)
    def update(self):
        if WORLD.FreeTesters == "Both" or WORLD.FreeTesters == "Tester 1":
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
        if WORLD.FreeTesters == "Both" or WORLD.FreeTesters == "Tester 2":
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
        if abs(WORLD.voltageReading - NORMAL_CELL_VOLTAGE) <= ALLOWED_VOLTAGE_TOLERANCE:
            self.feedback_message = f"Cell voltage {WORLD.voltageReading}V is within acceptable range."
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"Cell voltage {WORLD.voltageReading}V is out of acceptable range."
            return py_trees.common.Status.FAILURE
        
class AGVAtAssembly(py_trees.behaviour.Behaviour):
    # Check if any AGV is at the Assembly Station
    def __init__(self, name="Check if any AGV is at Assembly Station"):
        super().__init__(name)
    def update(self):
        if (WORLD.AGV1Location == "Assembly" or
            WORLD.AGV2Location == "Assembly" or
            WORLD.AGV3Location == "Assembly"):
            self.feedback_message = "An AGV is at the Assembly Station."
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "No AGV is at the Assembly Station."
            return py_trees.common.Status.FAILURE

class AssemblyAGVSlotsEmpty(py_trees.behaviour.Behaviour):
    # Check if any AGV at the Assembly Station has empty slots
    def __init__(self, name="Check for empty slots in AGVs at Assembly Station"):
        super().__init__(name)
    def update(self):
        agv_locations = [WORLD.AGV1Location, WORLD.AGV2Location, WORLD.AGV3Location]
        for x in range(0,3):
            if agv_locations[x] == "Assembly":
                for slot in WORLD.AGVSlots[x]:
                    if slot != " ":
                        self.feedback_message = f"AGV {x+1} at Assembly is not empty."
                        return py_trees.common.Status.FAILURE
                return py_trees.common.Status.SUCCESS