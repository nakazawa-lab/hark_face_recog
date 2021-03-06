from harkpython import harkbasenode


class HarkDebug(harkbasenode.HarkBaseNode):
    def __init__(self):
        print("-!!!!HarkDebug!!!!-" * 3)
        self.outputNames = ("OUTPUT",)
        self.outputTypes = ("prim_float",)
        self.c = 0

    def calculate(self):
        self.outputValues["OUTPUT"] = 1
        print("=" * 14 + str(type(self.INPUT)) + "=" * 14)
        print(self.INPUT)
        print("frame no." + str(self.c))
        self.c = self.c + 1
        print("")
