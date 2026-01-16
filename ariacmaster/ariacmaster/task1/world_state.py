import py_trees
import action_nodes.competition_behaviors as competition
import action_nodes.inspection_behaviors as inspection
import action_nodes.agv_behaviors as agv
import action_nodes.ir1_behaviors as ir1
import action_nodes.ir2_behaviors as ir2
import condition_nodes as condition
from core.ARIAC import Report
from core.types import DoorStates, IRLocations, Testers, AGVs, AGVLocations, Cells, Slots

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
    
    def create_behavior_tree(self):
        # Create the behavior tree:
        root = py_trees.composites.Sequence(
        name="Root",
        memory=False  # Re-evaluate from first child each tick
        )

        parallel_tasks = py_trees.composites.Parallel(
            name="Parallel Tasks",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
        )

        inspection = py_trees.composites.Sequence(
            name="Inspection",
            memory=False
        )

        cell_check = py_trees.composites.Selector(
            name="Cell Check",
            memory=False
        )

        conveyor_pickup = py_trees.composites.Sequence(
            name="Conveyor Pickup",
            memory=False
        )

        wait_for_queue = py_trees.composites.Selector(
            name="Wait for Queue",
            memory=False
        )

        wait_sequence = py_trees.composites.Sequence(
            name="Wait Sequence",
            memory=False
        )

        ir1_tester_selection = py_trees.composites.Selector(
            name="IR1 Tester Selection",
            memory=False
        )

        ir1_tester1_sequence = py_trees.composites.Sequence(
            name="IR1 Tester 1 Sequence",
            memory=False
        )
        
        ir1_tester2_sequence = py_trees.composites.Sequence(
            name="IR1 Tester 2 Sequence",
            memory=False
        )

        voltage_inspection = py_trees.composites.Sequence(
            name="Voltage Inspection",
            memory=False
        )

        check_testers_and_wait = py_trees.composites.Selector(
            name="Check Testers and Wait",
            memory=False
        )

        ir2_tester_selection = py_trees.composites.Selector(
            name="IR2 Tester Selection",
            memory=False
        )

        ir2_check_tester1 = py_trees.composites.Sequence(
            name="IR2 Check Tester 1",
            memory=False
        )

        ir2_check_tester2 = py_trees.composites.Sequence(
            name="IR2 Check Tester 2",
            memory=False
        )

        recycle_or_keep_cell_selector = py_trees.composites.Selector(
            name="Recycle or Keep Cell",
            memory=False
        )

        take_good_cell_to_agv = py_trees.composites.Sequence(
            name="Take Good Cell to AGV",
            memory=False
        )

        dispose_bad_cell = py_trees.composites.Sequence(
            name="Dispose Bad Cell",
            memory=False
        )

        agv_movement = py_trees.composites.Parallel(
            name="AGV Movement",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
        )

        get_agv_to_assembly = py_trees.composites.Sequence(
            name="Get AGV to Assembly",
            memory=False
        )

        return_agv_to_inspection = py_trees.composites.Sequence(
            name="Return AGV to Inspection",
            memory=False
        )


        # Decorators/Inverters added here... (do later)


        # Build the tree by adding children to parents here... (do later)



world = WorldState()
# Prevents imports from running this check
if __name__ == "__main__":
    print("World State Initialized:")
    print(world)
