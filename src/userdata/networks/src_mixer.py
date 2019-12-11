from harkpython import harkbasenode
import math
import os
import datetime
from pathlib import Path
import copy

class HarkNode(harkbasenode.HarkBaseNode):
    def __init__(self):
        print("=-" * 30 )
        self.outputNames=("OUTPUT",)  # one output terminal named "output"
        self.outputTypes=("vector_source",)
        self.flag = 0
        self.tmp_src = []

    def calculate(self):
        self.output_srcs = []
        # self.set_src(self.SRC_MUSIC)
        if len(self.SRC_ROS)!=0:
            self.tmp_src = self.SRC_ROS
        # print(self.SRC_ROS)
        self.set_src(self.tmp_src)
        for s in self.SRC_OFFSET:
            s['id'] += 20000
        self.set_src(self.SRC_OFFSET)
        print(self.outputValues["OUTPUT"])
        self.outputValues["OUTPUT"] = self.output_srcs

    def set_src(self, srcs):
        if len(srcs)!=0:
            for src in srcs:
                if src['power'] == 0:
                    continue
                self.output_srcs.append(src)


