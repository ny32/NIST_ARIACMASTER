from typing import Literal
#String Literal Types
DoorStates = Literal["Open", "Closed"]
IRLocations = Literal["Conveyor", "Tester 1", "Tester 2", "Recycling Bin", "AGV 1", "AGV 2", "AGV 3"]
Testers = Literal["Both", "Tester 1", "Tester 2", "None"]
AGVs = Literal["AGV 1", "AGV 2", "AGV 3"]
AGVLocations = Literal["Inspection", "Assembly", "Shipping", "Recycling", "Intersection"]
Defects = Literal["None", "Dent", "Scratch", "Bulge"]
Cells = Literal["Cell 1", "Cell 2", "None"]
Slots = Literal[" ", "X"]
#String Literal Types

# NIST ARIAC Definitions
NORMAL_CELL_VOLTAGE = 12.7 # Normal Voltage for a real cell
ALLOWED_VOLTAGE_TOLERANCE = 0.2 # NIST ARIAC - Allowed voltage tolerance for a cell
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
        self.IR2TargetCell: Cells = "None"

        self.FreeTesters: Testers = "Both"
        self.cellsDisposed: int = 0
        self.cellsKitted: int = 0
        self.cellsQueued: int = 0
        self.voltageReading: float = 0.0
        self.FreeAGVSlot: list[int] = [0, 0]

        self.filledAGVs: list[AGVs] = []

        self.AGVSlots: list[list[Slots]] = [
            [
                " ", " ", " ", " "
            ], # AGV 1
            [
                " ", " ", " ", " "
            ], #AGV 2
            [
                " ", " ", " ", " "
            ] # AGV 3
        ]

        self.AGV1Location: AGVLocations = "Inspection"
        self.AGV2Location: AGVLocations = "Inspection"
        self.AGV3Location: AGVLocations = "Inspection"

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

# Prevents imports from running this check
if __name__ == "__main__":
    print("World State Initialized:")
    print(world)
