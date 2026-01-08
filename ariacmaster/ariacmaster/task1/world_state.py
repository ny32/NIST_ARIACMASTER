import py_trees
from typing import Literal

# NIST ARIAC Definitions
class Report:
    def __init__(self, Status, DefectType):
        self.Status: bool = False
        self.DefectType: str = ""
# NIST ARIAC Definitions

#String Literal Types
DoorStates = Literal["Open", "Closed"]
IRLocations = Literal["Conveyor", "Tester 1", "Tester 2", "Recycling Bin", "AGV 1", "AGV 2", "AGV 3"]
Testers = Literal["Both", "Tester 1", "Tester 2", "None"]
AGVs = Literal["AGV 1", "AGV 2", "AGV 3", "None"]
AGVLocations = Literal["Inspection", "Assembly", "Shipping", "Recycling"]
#String Literal Types

class WorldState:
    def __init__(self):
        #Green
        self.Report: Report = Report(False, "")
        self.InspectionDoor: DoorStates = "Closed"
        
        #Purple
        self.IR1Free: bool = True
        self.IR1Grasping: bool = False
        self.IR1Location: IRLocations = "Conveyor"
        self.FreeTesters: Testers = "Both"

        #Blue
        self.IR2Free: bool = True
        self.IR2Grasping: bool = False
        self.IR2Location: IRLocations = "Conveyor"

        #Pink
        self.AGVPriority: AGVs = "None"
        
        self.cellsDisposed: int = 0
        self.cellsKitted: int = 0

