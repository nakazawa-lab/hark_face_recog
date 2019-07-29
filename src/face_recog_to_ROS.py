#!/usr/bin/env python
## coding: UTF-8

# ros系のライブラリ
import roslib
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CameraInfo

# ros以外
import cv2
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError
import imutils
from imutils import face_utils

# 自作モジュール
import dlib_module as dm

class SendFaceToROS:
    def __init__(self):
        # 顔認識に関する記述
        self.predictor_path = "./shape_predictor_68_face_landmarks.dat"
        self.face = dm.FaceDLib(self.predictor_path)
        # ROSのメソッド
        self._face_recog_pub = rospy.Publisher('face_recog_result', Float32MultiArray, queue_size=10)
        self._image_sub = rospy.Subscriber('/kinect2/hd/image_color', Image, self.callback)
        # OpenCVのメソッド
        self._bridge = CvBridge()
        # kinectカメラの情報を取得
        camera_info = rospy.wait_for_message("/kinect2/sd/camera_info", CameraInfo)
        self.width = camera_info.width
        self.height = camera_info.height
        K = np.array(camera_info.K).reshape(3,3) # 参照：http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        self.f = K[0][0] # 焦点距離f

    def send_to_ROS(self, x, y, z):
        array = Float32MultiArray(data=[x, y, z])
        self._face_recog_pub.publish(array)

    def acquire_face_angle(self, u):
        radian = math.atan((u - self.width/2) / self.f)
        theta = np.rad2deg(radian)
        return theta

    def theta_debug(self, theta):
        radian = np.radians(theta)
        u = math.tan(radian) * self.f + self.width/2
        return u

    def callback(self, data):
        cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        cv_image = imutils.resize(cv_image, self.width)

        # デバッグ用表示
        display_image = self.face.face_shape_detector_display(cv_image)
        cv2.imshow('img', display_image)
        cv2.waitKey(1)

        face_recog_result = self.face.get_mouth_xy(cv_image)

        if not face_recog_result == None:
            u = face_recog_result[0]
            v = face_recog_result[1]
            theta = self.acquire_face_angle(u)
            print("face angle: " + str(theta) + " [degree]")

            x = (u - self.width/2)
            y = (v - self.height/2)
            z = self.f
            self.send_to_ROS(x, y, z)
        
if __name__ == "__main__":
    rospy.init_node('face_recog_to_ROS',anonymous=True)
    face_recognition = SendFaceToROS()
    rospy.spin()
