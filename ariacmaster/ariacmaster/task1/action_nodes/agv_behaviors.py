import py_trees
import time
from world.main import WORLD


class FindNearestFreeAGVSlot(py_trees.behaviour.Behaviour):
    def __init__(self, name="Locate Nearest Cell"):
        super().__init__(name)
    
    def update(self):
        for AGV in range(0,3):
            for Slot in range(0, 4):
                if WORLD.AGVSlots[AGV][Slot] != "X":
                    WORLD.FreeAGVSlot = [AGV, Slot]
                    self.feedback_message = f"Found available slot {Slot} on AGV {AGV}."
                    return py_trees.common.Status.SUCCESS
        self.feedback_message = "No available AGV slots at this time."
        return py_trees.common.Status.FAILURE

class LocateFilledInspectionAGVs(py_trees.behaviour.Behaviour):
    def __init__(self, name="Check if Inspection AGVs have filled slots"):
        super().__init__(name)
    def update(self):
        for AGV in range(0, 3):
            slotsFilled = True
            for slots in range (0, 4):
                if WORLD.AGVSlots[slots] != "X":
                    slotsFilled = False
                    break
            if slotsFilled:
                match AGV:
                    case 0:
                        AGVName = "AGV 1"
                    case 1:
                        AGVName = "AGV 2"
                    case 2:
                        AGVName = "AGV 3"
                if AGVName not in WORLD.filledAGVs:
                    WORLD.filledAGVs.append(AGVName)
                    self.feedback_message = f"{AGVName} is filled and ready to move."
        return py_trees.common.Status.SUCCESS

class MoveAGVsToIntersection(py_trees.behaviour.Behaviour):
    def __init__(self, name="Move filled AGVs to intersection"):
        super().__init__(name)
    def update(self):
        for AGV in WORLD.filledAGVs:
            match AGV:
                case "AGV 1":
                    if WORLD.AGV1Location != "Intersection":
                        self.feedback_message = "Moving AGV 1 to Intersection"
                        # Simulate movement time and progress
                        self.feedback_message = "AGV 1 in transit..."
                        time.sleep(2)
                        self.feedback_message = "AGV 1 reached Intersection"
                case "AGV 2":
                    if WORLD.AGV2Location != "Intersection":
                        self.feedback_message = "Moving AGV 2 to Intersection"
                        self.feedback_message = "AGV 1 in transit..."
                        time.sleep(2)
                        self.feedback_message = "AGV 2 reached Intersection"
                case "AGV 3":
                    if WORLD.AGV3Location != "Intersection":
                        self.feedback_message = "Moving AGV 3 to Intersection"
                        self.feedback_message = "AGV 1 in transit..."
                        time.sleep(2)
                        self.feedback_message = "AGV 3 reached Intersection"
        return py_trees.common.Status.SUCCESS

class Wait1Sec(py_trees.behaviour.Behaviour):
    def __init__(self, name="Wait 1 Second"):
        super().__init__(name)
    def update(self):
        self.feedback_message = "Waiting 1 second"
        time.sleep(1)
        self.feedback_message = "Done!"
        return py_trees.common.Status.SUCCESS
    
class MoveAGVToAssembly(py_trees.behaviour.Behaviour):
    def __init__(self, name="Move AGV to Assembly Station"):
        super().__init__(name)
    def update(self):
        for AGV in WORLD.filledAGVs:
            match AGV:
                case "AGV 1":
                    if WORLD.AGV1Location == "Intersection":
                        self.feedback_message = "Moving AGV 1 to Assembly Station"
                        # Simulate movement time and progress
                        self.feedback_message = "AGV 1 in transit..."
                        time.sleep(2)
                        WORLD.AGV1Location = "Assembly"
                        self.feedback_message = "AGV 1 reached Assembly Station"
                case "AGV 2":
                    if WORLD.AGV2Location == "Intersection":
                        self.feedback_message = "Moving AGV 2 to Assembly Station"
                        self.feedback_message = "AGV 2 in transit..."
                        time.sleep(2)
                        WORLD.AGV2Location = "Assembly"
                        self.feedback_message = "AGV 2 reached Assembly Station"
                case "AGV 3":
                    if WORLD.AGV3Location == "Intersection":
                        self.feedback_message = "Moving AGV 3 to Assembly Station"
                        self.feedback_message = "AGV 3 in transit..."
                        time.sleep(2)
                        WORLD.AGV3Location = "Assembly"
                        self.feedback_message = "AGV 3 reached Assembly Station"
        return py_trees.common.Status.SUCCESS
class MoveAssemblyAGVToInspection(py_trees.behaviour.Behaviour):
    def __init__(self, name="Move Assembly AGV to Inspection Station"):
        super().__init__(name)
    def update(self):
        agv_locations = [WORLD.AGV1Location, WORLD.AGV2Location, WORLD.AGV3Location]
        for x in range(0,3):
            if agv_locations[x] == "Assembly":
                self.feedback_message = f"Moving AGV {x+1} to Inspection Station"
                time.sleep(2) # Simulate movement time
                # Bypasses intersection because all vehicles will yield to assembly vehicles
                match x:
                    case 0:
                        WORLD.AGV1Location = "Inspection"
                    case 1:
                        WORLD.AGV2Location = "Inspection"
                    case 2:
                        WORLD.AGV3Location = "Inspection"
                self.feedback_message = f"AGV {x+1} reached Inspection Station"
        return py_trees.common.Status.SUCCESS