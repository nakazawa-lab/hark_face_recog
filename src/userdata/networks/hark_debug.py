from harkpython import harkbasenode


class HarkDebug(harkbasenode.HarkBaseNode):
    def __init__(self):
        print("-!!!!HarkDebug!!!!-" * 3)
        self.outputNames = ("OUTPUT",)
        self.outputTypes = ("prim_float",)

    def calculate(self):
        print("=" * 30)
        print(self.INPUT)
        self.outputValues["OUTPUT"] = 1;