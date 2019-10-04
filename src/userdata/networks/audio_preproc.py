from harkpython import harkbasenode

class PreProc(harkbasenode.HarkBaseNode):
    def __init__(self):
        print("=pre=" * 100 )
        self.outputNames=("AUDIOOUT",)  # one output terminal named "output"
        self.outputTypes=("matrix_float",)  # the type is primitive float.

    def calculate(self):
        print("debug===================== " + str(self.AUDIOIN))
        self.outputValues["AUDIOOUT"] = self.AUDIOIN