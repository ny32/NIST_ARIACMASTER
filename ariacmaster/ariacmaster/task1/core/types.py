from typing import Literal

DoorStates = Literal["Open", "Closed"]
IRLocations = Literal["Conveyor", "Tester 1", "Tester 2", "Recycling Bin", "AGV 1", "AGV 2", "AGV 3"]
Testers = Literal["Both", "Tester 1", "Tester 2", "None"]
AGVs = Literal["AGV 1", "AGV 2", "AGV 3"]
AGVLocations = Literal["Inspection", "Assembly", "Shipping", "Recycling", "Intersection"]
Defects = Literal["None", "Dent", "Scratch", "Bulge"]
Cells = Literal["Cell 1", "Cell 2", "None"]
Slots = Literal[" ", "X"]