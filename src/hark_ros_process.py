from harkpython import harkbasenode

class HarkNode(harkbasenode.HarkBaseNode):
    def __init__(self):
        print("=" * 30 )
        self.outputNames=("OUTPUT",)  # one output terminal named "output"
        #self.outputTypes=("prim_float",)  # the type is primitive float.

        self.outputTypes=("vector_source",)

    def calculate(self):

        if len(self.SOURCES)<1:
            print("sound_source: no data")
        else:
            #print(self.SOURCES[0])
            print("sound_source:", self.SOURCES)


        if len(self.SOURCES2)<1:
            print("image_source: no data")
        else:
            #print(self.SOURCES[0])
            print("image_source:", self.SOURCES2)

        #出力の値を設定
        #self.outputValues["OUTPUT"] = [self.SOURCE[0]]
        #self.outputValues["OUTPUT"] = [] if len(self.SOURCES)<1 else [self.SOURCES[0]]
        self.outputValues["OUTPUT"] = [] if len(self.SOURCES2)<1 else [self.SOURCES2[0]]

        #定数入力は下記のように行う
        #self.outputValues["OUTPUT"] = [{'x': [-0.9433000087738037, -0.1662999987602234, 0.2874000072479248], 'power': 34.90971374511719, 'id': 2}]
