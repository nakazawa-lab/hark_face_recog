from harkpython import harkbasenode
import math
import os
import datetime
from pathlib import Path

class Spec2Array(harkbasenode.HarkBaseNode):
    def __init__(self):
        print("a" * 30 )
        self.outputNames=("OUTPUT",)  # one output terminal named "output"
        self.outputTypes=("matrix_float")  # the type is primitive float.
        # self.outputTypes=("vector_source",)
        # self.sources_container = []
        # self.flag = 0
        #
        # # EmitSourceLog関連
        # self.maxid = -1
        # # path = os.getcwd()
        # os.path.abspath(os.curdir)
        # os.chdir("..")
        # path = os.path.abspath(os.curdir)
        # self.f = open(str(path) + "/records/sournd_source_log/log_" + str(datetime.datetime.now()) + ".txt", "a")
        # os.chdir("networks")

    def calculate(self):
        print("sakaba: " + self.SPEC)
        self.outputValues["OUTPUT"] = self.SPEC
        # print(self.SOURCES2)
        # if len(self.SOURCES2)<1:
        #     # print("顔方向データがありません")
        #     if self.flag == 1:
        #         if len(self.SOURCES)<1:
        #             self.outputValues["OUTPUT"] = self.sources_container
        #         else:
        #             self.outputValues["OUTPUT"] = self.sources_container + self.SOURCES
        #     else:
        #         if len(self.SOURCES)<1:
        #             self.outputValues["OUTPUT"] = []
        #         else:
        #             self.outputValues["OUTPUT"] = self.SOURCES
        #
        #
        # elif self.SOURCES2[0]['power'] == 0:
        #     # print("顔方向データがありません")
        #     self.flag = 0
        #
        #     if len(self.SOURCES)<1:
        #         self.outputValues["OUTPUT"] = []
        #     else:
        #         self.outputValues["OUTPUT"] = self.SOURCES
        #
        # else:
        #     # print(self.SOURCES[0])
        #     # print("image_source:", self.SOURCES2)
        #     self.sources_container = [self.SOURCES2[0]]
        #     self.flag = 1
        #
        #     if len(self.SOURCES)<1:
        #         self.outputValues["OUTPUT"] = self.sources_container
        #     else:
        #         self.outputValues["OUTPUT"] = self.sources_container + self.SOURCES
        #
        # self.EmitSourceLog(self.outputValues["OUTPUT"])
        # # print(self.outputValues["OUTPUT"])
