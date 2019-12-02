#!/usr/bin/env python
## coding: UTF-8

import rospy
import numpy as np
import os
import cv2

from hark_msgs.msg import HarkWave
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import multichannel_wav_reader as mcwr

class MultiModal:
    def __init__(self):
        self.frame_no = 0
        np.set_printoptions(threshold=0)
        self._bridge = CvBridge()
        self._movie_data_pub = rospy.Publisher('moviedata_py', Image, queue_size=10)
        self._HarkWave_count_sub = rospy.Subscriber('wavdata_py', HarkWave, self.count_callback)
        # self._image_debug_sub = rospy.Subscriber('moviedata_py', Image, self.img_debug_callback)

        self.here_path = os.path.dirname(__file__)
        if self.here_path == "":
            self.here_path = "."

    def set_audio_sampling_rate(self, rate):
        self.audio_sr = rate
        self.sample_threshold = rate / self.fps

    def read_frames(self, movie_path):
        inputs = os.listdir(movie_path)
        self.cap = cv2.VideoCapture(movie_path + '/' + inputs[0])
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.duration = self.cap.get(cv2.CAP_PROP_FRAME_COUNT) / self.cap.get(cv2.CAP_PROP_FPS)
        print("frame per second:", self.fps)
        self.frames = []
        ret, cv_img = self.cap.read()
        while ret:
            self.frames.append(cv_img)
            ret, cv_img = self.cap.read()

    def count_callback(self, hw):
        imgs = self.frames
        audio_samples = (hw.count + 1) * hw.length
        if audio_samples > (self.frame_no + 1) * self.sample_threshold:
            msg = self._bridge.cv2_to_imgmsg(imgs[self.frame_no], 'bgr8')
            self._movie_data_pub.publish(msg)
            self.frame_no = self.frame_no + 1

    def img_debug_callback(self, data):
        cv_img = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        cv2.namedWindow("image")
        cv2.imshow("image", cv_img)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('multimodal_reader_py',anonymous=True)

    # 音声データに関する処理
    audio = mcwr.MultiWav()
    wavfiles = audio.read_input_dir(audio.here_path + "/input/wavfile")
    audio.initialize_HarkWave()
    audio_arr = audio.wav2array(wavfiles)

    # 動画データに関する処理
    movie = MultiModal()
    movie.read_frames(audio.here_path + "/input/mp4file")
    movie.set_audio_sampling_rate(audio.sampling_rate)
    audio.generate_senddata_with_playing(audio_arr)