from harkpython import harkbasenode
import math
import os
import datetime
from pathlib import Path


class HarkDebug(harkbasenode.HarkBaseNode):
    def __init__(self):
        print("-HarkDebug-" * 3)
        self.outputNames = ("OUTPUT",)
        self.outputTypes = ("prim_float",)

    def calculate(self):
        print("=" * 30)
        print(self.INPUT)
        print("=" * 30)
        self.outputValues["OUTPUT"] = 1;