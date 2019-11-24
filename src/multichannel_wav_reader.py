#!/usr/bin/env python
## coding: UTF-8

import pyaudio
import wave
import sys
import rospy
import numpy as np
import os
import copy
from std_msgs.msg import Int32MultiArray
from time import sleep

class MultiWav:
    def __init__(self):
        np.set_printoptions(threshold=0)
        # self._wav_data_pub = rospy.Publisher('wavdata_py', Int32MultiArray, queue_size=10)
        self.CHUNK = 512
        self.multi_msgs = []
        self.p = pyaudio.PyAudio()
        print("-=" * 35)

        self.here_path = os.path.dirname(__file__)
        if self.here_path == "":
            self.here_path = "."

    def read_input_dir(self, path):
        inputs = os.listdir(path)
        wf_list = []
        for input in inputs:
            input_path = path + "/" + input
            wf = wave.open(input_path, 'rb')
            wf_list.append(wf)
        # 他のメソッドで使用するメンバ変数等を定義
        self.channel0_path = path + "/" + inputs[0]
        self.pub_array = []
        for index in range(len(inputs)):
            topicname = 'wav_ch' + str(index)
            pub = rospy.Publisher(topicname, Int32MultiArray, queue_size=1)
            self.pub_array.append(pub)
            self.test_publish(pub)
        return wf_list

    def test_publish(self, pub):
        msg = Int32MultiArray(data=[5])
        for i in range(10):
            pub.publish(msg)
            sleep(0.05)

    def wav2array(self, wfs):
        for wf in wfs:
            data = wf.readframes(self.CHUNK)
            msgs_nest = []
            while data != '':
                # stream.write(data)
                data = wf.readframes(self.CHUNK)
                wavdata = np.fromstring(data, "Int16").tolist()
                if len(wavdata) == 0:
                    break
                msgs_nest.append(wavdata)
            self.multi_msgs.append(msgs_nest)

    def generate_senddata_with_playing(self, multi_msgs):
        wf0 = wave.open(self.channel0_path, 'rb')
        stream = self.p.open(
            format=self.p.get_format_from_width(wf0.getsampwidth()),
            channels=wf0.getnchannels(),
            rate=wf0.getframerate(),
            output=True
        )
        data = wf0.readframes(self.CHUNK)
        count = 0
        while data != '':
            stream.write(data)
            data = wf0.readframes(self.CHUNK)
            wavdata = np.fromstring(data, "Int16")
            print("playing this now: " + str(wavdata))
            if len(wavdata) == 0:
                break
            multi_msg_now = []
            for mono_msgs in multi_msgs:
                multi_msg_now.append(mono_msgs[count])
            self.send(multi_msg_now)
            count = count + 1
        stream.close()
        self.p.terminate()

    def send(self, multi_msg_now):
        for i, pub in enumerate(self.pub_array):
            msg = Int32MultiArray(data=multi_msg_now[i])
            pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('wav_reader_py',anonymous=True)
    mw = MultiWav()
    wavfiles = mw.read_input_dir(mw.here_path + "/input")
    mw.wav2array(wavfiles)
    mw.generate_senddata_with_playing(mw.multi_msgs)
