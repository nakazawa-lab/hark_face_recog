#!/usr/bin/env python
## coding: UTF-8


import pyaudio
import wave
import sys
import rospy
import numpy as np
from std_msgs.msg import Int8MultiArray
# from hark_msgs.msg import HarkWave
# from hark_msgs.msg import HarkWaveVal
# from _HarkWave import HarkWave
# import HarkWaveVal


class Send_wave_to_ros:
    def __init__(self):
        # インスタンス生成
        self.dhw = HarkWave()
        # フレームのカウント初期化
        self.count = 0



if __name__ == '__main__':

    CHUNK = 1024

    # wavデータ読み込み
    wav_file = "/home/nvidia/catkin_ws/src/hark_face_recog/src/userdata/records/movie_dataset/dataset2/multichannel_audio.wav"
    wf = wave.open(wav_file, 'rb')
    p = pyaudio.PyAudio()

    stream = p.open(
        format=p.get_format_from_width(wf.getsampwidth()),
        channels=wf.getnchannels(),
        rate=wf.getframerate(),
        # input=True,
        output=True
        # input_device_index=5
    )
    data = wf.readframes(CHUNK)
    # 読み込んだwavデータを数値に変換
    wavedata = np.fromstring(data, dtype=np.int8)
    print("wavedata", wavedata)

    sender = Send_wave_to_ros()

    sender.dhw = HarkWave()
    sender.dhw.header.stamp = rospy.Time.now()
    sender.dhw.header.frame_id = str(sender.dhw.count)
    sender.dhw.count = sender.count
    harkwaveval = HarkWaveVal(wavedata=data_uint8)
    result = []
    result.append(harkwaveval)
    sender.dhw.src = result
    sender.dhw.nch = 8  # one channel
    sender.dhw.length = 320
    sender.dhw.data_bytes = 320 * 4  # float(4bytes) times length of data


    # close PyAudio (5)
    p.terminate()

    while not rospy.is_shutdown():
        rospy.init_node('multichannel_wav_reader') # ノードの生成
        wave_pub = rospy.Publisher('HarkWave', HarkWave, queue_size=10) # chatterという名前のTopicを生成し型やらを定義
        rate = rospy.Rate(10) # 10Hzで動かすrateというクラスを生成

        wave_pub.publish(sender.dhw)
