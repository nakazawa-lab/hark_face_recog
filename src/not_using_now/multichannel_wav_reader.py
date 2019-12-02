#!/usr/bin/env python
## coding: UTF-8

import pyaudio
import wave
import rospy
import numpy as np
import os
from hark_msgs.msg import HarkWave, HarkWaveVal
from time import sleep

class MultiWav:
    def __init__(self):
        np.set_printoptions(threshold=0)
        self._wav_data_pub = rospy.Publisher('wavdata_py', HarkWave, queue_size=10)
        self.CHUNK = 512
        self.p = pyaudio.PyAudio()
        print("-=" * 35)
        self.count = 0

        self.here_path = os.path.dirname(__file__)
        if self.here_path == "":
            self.here_path = "."

    def initialize_HarkWave(self):
        self.hw = HarkWave()
        self.hwv = HarkWaveVal()
        self.sent_flag = False
        self.hw.length = 512
        self.hw.nch = self.ch
        self.hw.data_bytes = self.hw.length * self.hw.nch * 4

    def read_input_dir(self, path):
        inputs = os.listdir(path)
        wf_list = []
        for input in inputs:
            input_path = path + "/" + input
            wf = wave.open(input_path, 'rb')
            wf_list.append(wf)
        # 他のメソッドで使用するメンバ変数等を定義
        self.channel0_path = path + "/" + inputs[0]
        self.ch = len(inputs)
        self.sampling_rate = wf_list[0].getframerate()
        self.duration = float(wf_list[0].getnframes()) / wf_list[0].getframerate()
        return wf_list

    def wav2array(self, wfs):
        multi_msgs = []
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
            multi_msgs.append(msgs_nest)
        return multi_msgs

    def generate_senddata_with_playing(self, multi_msgs):
        wf0 = wave.open(self.channel0_path, 'rb')
        stream = self.p.open(
            format=self.p.get_format_from_width(wf0.getsampwidth()),
            channels=wf0.getnchannels(),
            rate=wf0.getframerate(),
            output=True
        )
        data = wf0.readframes(self.CHUNK)

        while data != '':
            stream.write(data)
            data = wf0.readframes(self.CHUNK)
            wavdata = np.fromstring(data, "Int16")
            print("playing frame" + str(self.count) + ": " + str(wavdata))
            if len(wavdata) == 0:
                break
            multi_msg_1d = []
            for mono_msgs in multi_msgs:
                multi_msg_1d.extend(mono_msgs[self.count])
            self.send(multi_msg_1d)
            self.count = self.count + 1
        stream.close()
        self.p.terminate()

    def send(self, array):
        hw = self.hw
        hwv = self.hwv
        hw.header.stamp = rospy.Time.now()
        hw.header.frame_id = str(self.count)
        hw.count = self.count
        hw.src = []
        for i in range(hw.nch):
            start_index = i * hw.length
            end_index = start_index + hw.length
            mono_arr = array[start_index:end_index]
            hwv.wavedata = mono_arr
            hw.src.append(hwv)
        self._wav_data_pub.publish(hw)

if __name__ == '__main__':
    rospy.init_node('wav_reader_py',anonymous=True)
    mw = MultiWav()
    wavfiles = mw.read_input_dir(mw.here_path + "/input/wavfile")
    mw.initialize_HarkWave()
    arr = mw.wav2array(wavfiles)
    mw.generate_senddata_with_playing(arr)
