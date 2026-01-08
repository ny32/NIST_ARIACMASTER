import py_trees
from py_trees import behaviours
from typing import Literal
import time
import random

#String Literal Types
DoorStates = Literal["Open", "Closed"]
IRLocations = Literal["Conveyor", "Tester 1", "Tester 2", "Recycling Bin", "AGV 1", "AGV 2", "AGV 3"]
Testers = Literal["Both", "Tester 1", "Tester 2", "None"]
AGVs = Literal["AGV 1", "AGV 2", "AGV 3", "None"]
AGVLocations = Literal["Inspection", "Assembly", "Shipping", "Recycling"]
Defects = Literal["None", "Dent", "Scratch", "Bulge"]
#String Literal Types

# NIST ARIAC Definitions
class Report:
    def __init__(self, Status, DefectType):
        self.Status: bool = False
        self.DefectType: Defects = "None"
# NIST ARIAC Definitions

class WorldState:
    def __init__(self):
        self.Report: Report = Report(False, "")
        self.InspectionDoor: DoorStates = "Closed"

        self.IR1Free: bool = True
        self.IR1Grasping: bool = False
        self.IR1Location: IRLocations = "Conveyor"

        self.IR2Free: bool = True
        self.IR2Grasping: bool = False
        self.IR2Location: IRLocations = "Conveyor"

        self.FreeTesters: Testers = "Both"
        self.AGVPriority: AGVs = "None" #Right of way for AGVs at intersections
        self.cellsDisposed: int = 0
        self.cellsKitted: int = 0
    def __str__(self):
        return (
            f"Inspection Door: {self.InspectionDoor}\n"
            f"Inspection Robot 1 Status: {'Free' if self.IR1Free else 'Busy'}\n"
            f"Inspection Robot 1 Location: {self.IR1Location}\n"
            f"Inspection Robot 2 Status: {'Free' if self.IR2Free else 'Busy'}\n"
            f"Inspection Robot 2 Location: {self.IR2Location}\n"
            f"-------------------------------------------\n"
            f"Cells Disposed/Kitted: {self.cellsDisposed}/{self.cellsKitted}"
        )
    

world = WorldState()
print("World State Initialized:")
print(world)

