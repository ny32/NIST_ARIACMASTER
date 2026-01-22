from core.ARIAC import Report
from core.types import DoorStates, IRLocations, Testers, AGVs, AGVLocations, Cells, Slots

class WorldState:
    def __init__(self):
        self.tick_count=0 # Counts ticks, not needed for ARIAC

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



# Prevents imports from running this check
if __name__ == "__main__":
    _world = WorldState()
    print("World State Initialized:")
    print(_world)
