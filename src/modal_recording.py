#!/usr/bin/env python
## coding: UTF-8

import rospy
import numpy as np
import os
import cv2
import atexit

from hark_msgs.msg import HarkWave
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge

class MultiModal:
    def __init__(self):
        self.vframe_no = 1
        self.advance = 160
        np.set_printoptions(threshold=0)
        self._bridge = CvBridge()
        self.FPS = 15
        self.SAMPLE_THRESH = 16000 / self.FPS

        self.here_path = os.path.dirname(__file__)
        if self.here_path == "":
            self.here_path = "."

        # 録画用の設定
        camera_info = rospy.wait_for_message("/kinect2/hd/camera_info", CameraInfo)
        save_path = self.here_path + '/userdata/records/sep_files/video.mp4'
        self.height = camera_info.height
        self.width = camera_info.width
        fourcc = cv2.VideoWriter_fourcc('m','p','4','v')
        self.writer = cv2.VideoWriter(save_path, fourcc, self.FPS, (self.width, self.height))

        self._HarkWave_count_sub = rospy.Subscriber('/HarkWave', HarkWave, self.count_callback)
        self._image_sub = rospy.Subscriber('/kinect2/hd/image_color', Image, self.image_callback)

    def count_callback(self, data):
        audio_frame_no = data.count
        audio_samples = (audio_frame_no + 1) * self.advance
        if audio_samples > self.SAMPLE_THRESH * self.vframe_no:
            self.writer.write(self.img)
            self.vframe_no += 1

    def image_callback(self, data):
        cv_img = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        self.img = cv_img
        fxy = 0.3
        cv_img= cv2.resize(cv_img, dsize=None, fx=fxy, fy=fxy)
        if len(cv_img) == 0:
            cv2.destroyWindow("image")
            exit()
        cv2.namedWindow("image")
        cv2.imshow("image", cv_img)
        cv2.waitKey(1)

    def all_done(self):
        print("")
        self.writer.release()
        print('record done!!!')

if __name__ == '__main__':
    rospy.init_node('movie_recorder',anonymous=True)

    # 動画データに関する処理
    movie = MultiModal()
    rospy.loginfo('running..')
    rospy.spin()
    atexit.register(movie.all_done)