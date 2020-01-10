#!/usr/bin/env python
## coding: UTF-8
from harkpython import harkbasenode

class HarkNode(harkbasenode.HarkBaseNode):
    def __init__(self):
        print("=-" * 30 )
        self.outputNames=("OUTPUT",)  # one output terminal named "output"
        self.outputTypes=("vector_source",)
        self.flag = 0
        self.tmp_src = []

    def calculate(self):
        # 最終的に送信する配列を用意
        self.output_srcs = []

        # 音源定位結果の反映
        for s in self.SRC_MUSIC:
            s['id'] += 100
        self.set_src(self.SRC_MUSIC)

        # ROSからの顔方向情報の反映
        if len(self.SRC_ROS)!=0:
            self.tmp_src = self.SRC_ROS
        self.set_src(self.tmp_src)
        print(self.output_srcs)

        # 角度を定数として扱う場合
        for s in self.SRC_OFFSET:
            s['id'] += 20000
        # self.set_src(self.SRC_OFFSET)

        # 下記でHARKの次ノードへ送信
        self.outputValues["OUTPUT"] = self.output_srcs

    # 定位情報を配列に加える関数
    def set_src(self, srcs):
        if len(srcs)!=0:
            for src in srcs:
                if src['power'] == 0:
                    continue
                self.output_srcs.append(src)


