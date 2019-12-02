#!/usr/bin/env python
## coding: UTF-8

# ros系のライブラリ
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CameraInfo

# ros以外
import cv2
import numpy as np
import math
import dlib
from cv_bridge import CvBridge, CvBridgeError
import imutils
import os
import atexit
import time
from time import sleep
import datetime
from statistics import mean, median,variance,stdev
import sys

# 自作モジュール
import dlib_module as dm

class DlibEx:
    def __init__(self):
        here_path = os.path.dirname(__file__)
        if here_path == "":
            here_path = "."
        self.predictor_path = here_path + "/shape_predictor_68_face_landmarks.dat"
        self.face = dm.FaceDLib(self.predictor_path)
        self._bridge = CvBridge()
        self.detector = dlib.get_frontal_face_detector()
        camera_info = rospy.wait_for_message("/kinect2/hd/camera_info", CameraInfo)
        r = 1
        self.width = int(camera_info.width * r)
        self.height = int(camera_info.height * r)
        self.MAR_THRESH = 0.70 # mouth aspect ratioの閾値(marの値がこの値以上になった場合口が開いていると判断する)
        self.f = open(here_path + "/userdata/records/dlib_ex/dl" + str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")) + ".txt", "a")
        self.f.write("isOpen|MAR|score" + "\n")
        self._image_sub = rospy.Subscriber('/kinect2/hd/image_color', Image, self.callback)
        self.is_open_flag = 0
        self.fps_array = [0, 5, 10, 15, 20, 25, 56, 52, 1, 100] # 初期値適当
        self.start = time.time()
        self.frame_no = 1
        self.rect_array = []

    # mouth_aspect_ratioを使用して口が動いているかを判定する
    def mouth_motion_with_mar(self, mar, flag):
        # 口が閉まっている場合カウントを1ずつ増やしていく
        if mar < self.MAR_THRESH:
            self.mouth_close_count += 1
        # 口が開いている場合にカウントを0にする
        else:
            self.mouth_close_count = 0
        print("mouth_close_count:", self.mouth_close_count)
        # カウントが10以上の場合人が話していないと判断する
        if self.mouth_close_count >= 10:
            self.start_flag = 1 # 1度カウントが10を超えたらフラグを立てて(1にして)、以後はカウントが10より小さい場合に口が動いていると判定する
            # print("話していません")
            return False
        else:
            if self.start_flag == 1:
                if self.speaking_flag == 0:
                    self.speaking_flag = 1
                # print("話しています")
                return True
            else:
                # print("話していません")
                return False

    def all_done(self):
        self.f.close()
        print("")
        # print("==================================================")
        # print("xy: " + str(min(self.rect_array)) + ", fps: " + str(self.fps) + ", image: " + str(self.image.shape))
        # print("==================================================")
        print("ex done!")

    def debug(self, d):
        print(d)

    # ROSによって繰り返し呼び出される
    def callback(self, data):
        cv_img = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        # cv_img = cv2.resize(cv_img, (self.width, self.height))
        cv_img = imutils.resize(cv_img, int(sys.argv[1]))
        self.image = cv_img
        img_gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        # 画像の中から顔を検出
        rects, scores, idx  = self.detector.run(img_gray, 0, 0)
        if len(scores) == 0:
            s = ""
        else:
            s = str(scores[0])
        # print ("rects: " + str(rects))

        # mouth aspect ratio(口の開き具合の指標)を取得
        mar = self.face.mouth_aspect_ratio(img_gray, rects)
        # print ()


        #
        # # デバッグ用表示
        display_image = self.face.face_shape_detector_display(cv_img, img_gray, rects,  mar, self.MAR_THRESH)
        if len(rects) != 0:
            rect = rects[0]
            cv2.rectangle(display_image, (rect.left(), rect.top()), (rect.right(), rect.bottom()), (255, 0, 0), 2)
            self.rect_array.append(rect.left()-rect.right())


        cv2.imshow('img', display_image)
        k = cv2.waitKey(1)
        if self.is_open_flag % 2 == 0:
            d = "Close"
            self.is_open = "FALSE"
        else:
            d = "OPEN"
            if self.is_open_flag == 1:
                self.is_open = "a"
            if self.is_open_flag == 3:
                self.is_open = "i"
            if self.is_open_flag == 5:
                self.is_open = "u"
            if self.is_open_flag == 7:
                self.is_open = "e"
            if self.is_open_flag == 9:
                self.is_open = "o"
        # self.debug(d)

        if k == 13: # エンター押したら
            self.is_open_flag = self.is_open_flag + 1
        self.f.write(str(self.is_open) + "|" + str(mar) + "|" + s + "\n")

        self.fps = self.frame_no / (time.time() - self.start)
        self.fps_array.append(self.fps)
        self.fps_array.pop(0)
        self.frame_no = self.frame_no + 1
        v = variance(self.fps_array)
        # print("v: " + str(v) + ", fps: " + str(self.fps))
        # if v < 10 ** (-4):
        #     print("==================================================")

if __name__ == "__main__":
    rospy.init_node('dlib_ex',anonymous=True)
    de = DlibEx()
    sleep(1)
    atexit.register(de.all_done)
    rospy.spin()
