#!/usr/bin/env python2
## coding: UTF-8

from harkpython import harkbasenode
import math
import os
import datetime
import rospy
from std_msgs.msg import String
from pathlib import Path

class HarkNode(harkbasenode.HarkBaseNode):
    def __init__(self):
        print("=-" * 30 )
        self.outputNames=("OUTPUT",)  # one output terminal named "output"
        #self.outputTypes=("prim_float",)  # the type is primitive float.
        self.outputTypes=("vector_source",)
        self.sources_container = []
        self.flag = 0

        # EmitSourceLog関連
        self.maxid = -1
        # path = os.getcwd()
        os.path.abspath(os.curdir)
        os.chdir("..")
        path = os.path.abspath(os.curdir)
        self.f = open(str(path) + "/records/sournd_source_log/log_" + str(datetime.datetime.now()) + ".txt", "a")
        os.chdir("networks")

        # ros関連
        # ノードの作成
        rospy.init_node('hark_ros_process_from_wav')
        self.start_sign_pub = rospy.Publisher('source_info', String, queue_size=10)


    def calculate(self):

        # if len(self.SOURCES)<1:
        #     print("音源データがありません")
        # else:
        #     print("sound_source:", self.SOURCES)

        # 音源定位結果がある場合、動画情報を読み込み始める合図をトピックとして送信
        if len(self.SOURCES) < 1:
            send_msg = "start"
            rospy.loginfo(send_msg)
            self.start_sign_pub.publisher(send_msg)



        print("face_localization_result:", self.SOURCES2)
        if len(self.SOURCES2)<1:
            # print("顔方向データがありません")
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
            # print("顔方向データがありません")
            self.flag = 0

            if len(self.SOURCES)<1:
                self.outputValues["OUTPUT"] = []
            else:
                self.outputValues["OUTPUT"] = self.SOURCES

        else:
            # print(self.SOURCES[0])
            # print("image_source:", self.SOURCES2)
            self.sources_container = [self.SOURCES2[0]]
            self.flag = 1

            if len(self.SOURCES)<1:
                self.outputValues["OUTPUT"] = self.sources_container
            else:
                self.outputValues["OUTPUT"] = self.sources_container + self.SOURCES

        self.EmitSourceLog(self.outputValues["OUTPUT"])
        # print(self.outputValues["OUTPUT"])

    
    def EmitSourceLog(self, srcs):
        if len(srcs)==0:
            return
        for src in srcs:
            if src['id'] > self.maxid:
                self.f.write(str(src) + "\n")
                self.maxid = src['id'] 