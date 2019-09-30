from harkpython import harkbasenode
import math

class HarkNode(harkbasenode.HarkBaseNode):
    def __init__(self):
        print("=" * 30 )
        self.outputNames=("OUTPUT",)  # one output terminal named "output"
        #self.outputTypes=("prim_float",)  # the type is primitive float.
        self.outputTypes=("vector_source",)
        self.sources_container = []
        self.flag = 0

    def calculate(self):

        if len(self.SOURCES)<1:
            print("音源データがありません")
        else:
            #print(self.SOURCES[0])
            print("sound_source:", self.SOURCES)


        # if len(self.SOURCES2)<1:
        #     print("顔方向データがありません")
        #     if self.flag == 1:
        #         self.outputValues["OUTPUT"] = self.sources_container
        #     else:
        #         self.outputValues["OUTPUT"] = []
        #
        # elif self.SOURCES2[0]['power'] == 0:
        #     print("顔方向データがありません")
        #     self.flag = 0
        #     self.outputValues["OUTPUT"] = []
        #
        # else:
        #     #print(self.SOURCES[0])
        #     print("image_source:", self.SOURCES2)
        #     self.sources_container = [self.SOURCES2[0]]
        #     self.flag = 1
        #     self.outputValues["OUTPUT"] = self.sources_container


        if len(self.SOURCES2)<1:
            print("顔方向データがありません")
            if self.flag == 1:
                if len(self.SOURCES)<1:
                    self.outputValues["OUTPUT"] = self.sources_container
                else:
                    self.outputValues["OUTPUT"] = self.sources_container + self.SOURCES
            else:
                if len(self.SOURCES)<1:
                    self.outputValues["OUTPUT"] = []
                else:
                    self.outputValues["OUTPUT"] = self.SOURCES


        elif self.SOURCES2[0]['power'] == 0:
            print("顔方向データがありません")
            self.flag = 0

            if len(self.SOURCES)<1:
                self.outputValues["OUTPUT"] = []
            else:
                self.outputValues["OUTPUT"] = self.SOURCES

        else:
            #print(self.SOURCES[0])
            print("image_source:", self.SOURCES2)
            self.sources_container = [self.SOURCES2[0]]
            self.flag = 1

            if len(self.SOURCES)<1:
                self.outputValues["OUTPUT"] = self.sources_container
            else:
                self.outputValues["OUTPUT"] = self.sources_container + self.SOURCES


        print(self.outputValues["OUTPUT"])


        #出力の値を設定
        #self.outputValues["OUTPUT"] = [self.SOURCE[0]]
        # self.outputValues["OUTPUT"] = [] if len(self.SOURCES)<1 else [self.SOURCES]
        #self.outputValues["OUTPUT"] = [] if len(self.SOURCES2)<1 else [self.SOURCES2[0]]
        #self.outputValues["OUTPUT"] = [] if len(self.SOURCES2)<1 or self.SOURCES2[0]['power'] == 0 else [self.SOURCES2[0]]




        #定数入力は下記のように行う
        #self.outputValues["OUTPUT"] = [{'x': [-0.9433000087738037, -0.1662999987602234, 0.2874000072479248], 'power': 34.90971374511719, 'id': 2}]
