#!/usr/bin/env python
## coding: UTF-8

# ros系のライブラリ
import roslib
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int64MultiArray

# ros以外
import cv2
import numpy
import time
from cv_bridge import CvBridge, CvBridgeError
import imutils
from imutils import face_utils

# 自作モジュール
import dlib_module as dm

class SendFaceToROS:
    def __init__(self):
        # メンバ変数
        self.predictor_path = "./shape_predictor_68_face_landmarks.dat"
        self.face = dm.FaceDLib(self.predictor_path)
        # メソッド
        self._face_recog_pub = rospy.Publisher('face_recog_result', Int64MultiArray, queue_size=10)
        self._bridge = CvBridge()
        self._image_sub = rospy.Subscriber('/kinect2/hd/image_color', Image, self.callback)

    def send_to_ROS(self, camera_x, camera_y):
        array = Int64MultiArray(data=[camera_x, camera_y])
        self._face_recog_pub.publish(array)

    def callback(self, data):
        cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        cv_image = imutils.resize(cv_image, width=512)
        # print(cv_image.shape)

        # デバッグ用表示
        display_image = self.face.face_shape_detector_display(cv_image)
        cv2.imshow('img', display_image)
        cv2.waitKey(1)

        face_recog_result = self.face.get_mouth_xy(cv_image)
        if not face_recog_result == None:
            x = face_recog_result[0]
            y = face_recog_result[1]
            self.send_to_ROS(x, y)
            print(x, y)

if __name__ == "__main__":
    rospy.init_node('face_recog_to_ROS',anonymous=True)
    face_recognition = SendFaceToROS()
    rospy.spin()
