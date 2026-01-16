from core.types import Defects

class Report:
    def __init__(self, Status, DefectType):
        self.Status: bool = False
        self.DefectType: Defects = "None"