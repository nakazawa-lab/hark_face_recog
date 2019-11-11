#!/usr/bin/env python
## coding: UTF-8

# ros系のライブラリ
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

# ros以外
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
import sys
import atexit
import time
from time import sleep
import datetime


class VWriter():
    def __init__(self):
        # kinectのカメラ情報
        camera_info = rospy.wait_for_message("/kinect2/hd/camera_info", CameraInfo)
        self.height = camera_info.height
        self.width = camera_info.width
        print("height: " + str(self.height) + ", width: " + str(self.width))
        # openCV
        self._bridge = CvBridge()
        # 録画用の設定
        fourcc = cv2.VideoWriter_fourcc('m','p','4','v')
        self.here_path = os.path.dirname(__file__)
        if self.here_path == "":
            self.here_path = "."

        save_path = self.here_path + "/userdata/records/video/v.mp4"
        self.fix_video_fps = float(sys.argv[1])
        self.writer = cv2.VideoWriter(save_path, fourcc, self.fix_video_fps, (self.width, self.height))
        self.start = time.time()
        self.frame_no = 1
        self.T = 0.061 # 周期を100msくらいにしたい
        self.fps = 0.0
        self.rec_flag = False
        # rospy
        self._image_sub = rospy.Subscriber('dsampled_image', Image, self.callback)

    def all_done(self):
        print ""
        self.writer.release()
        before = self.here_path + "/userdata/records/video/v.mp4"
        after = self.here_path + "/userdata/records/video/v" + str(self.rec_start) + ".mp4"
        os.rename(before, after)
        print 'record done!!!'

    # ROSによって繰り返し呼び出される
    def callback(self, data):
        t_0 = time.time()
        cv_img = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        if abs(self.fps - self.fix_video_fps) < 0.1:
            self.rec_flag = True
        if self.rec_flag == True:
            self.rec_start = str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))
            self.writer.write(cv_img)
        interval = time.time() - t_0
        if  interval < self.T:
            print("fps: " + str(self.fps) + " " + str(self.rec_flag))
            sleep(self.T - interval)
        self.fps = self.frame_no / (time.time() - self.start)
        self.frame_no = self.frame_no + 1


if __name__ == "__main__":
    rospy.init_node('dlib_ex',anonymous=True)
    vw = VWriter()
    atexit.register(vw.all_done)
    rospy.spin()
