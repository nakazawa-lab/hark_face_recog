#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import message_filters
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Int64MultiArray
import cv2
from cv_bridge import CvBridge, CvBridgeError
 
class FaceRecog:
    def __init__(self):
        self.bridge = CvBridge()
        self.face_uv = []
        self.u = 0
        self.v = 0
        self.sync_flag = 0

        # 画像情報取得のsubscriberの定義
        face_recog_sub = rospy.Subscriber('/face_recog_result', Int64MultiArray, self.callback, callback_args=0)
        depth_sub = rospy.Subscriber('/kinect2/qhd/image_depth_rect', Image, self.callback, callback_args=1)

        # カメラのパラメータの取得
        # invKは内部パラメータ(https://mem-archive.com/2018/02/21/post-157/)
        self.camera_info = rospy.wait_for_message("/kinect2/qhd/camera_info", CameraInfo)
        self.invK = np.linalg.inv(np.array(self.camera_info.K).reshape(3,3))

    # u,vは画像座標でx,y,zはカメラ座標
    # https://mem-archive.com/2018/02/21/post-157/
    def uv_to_xyz(self, depth_image, u, v):
        depth_array = np.array(depth_image, dtype=np.float32)

        info = self.camera_info
        image_point =np.array([u, v, 1])
        z = depth_array[u, v]
        xyz_point = np.matmul(self.invK, image_point) * z 

        x = round(xyz_point[0]/1000, 2)
        y = round(xyz_point[1]/1000, 2)
        z = round(xyz_point[2]/1000, 2)
        print (x,y,z)

    def display(self, frame):
        info = self.camera_info
        cv2.circle(frame, (info.width/2, info.height/2), 5, color=(255, 0, 0), thickness=2)
        cv2.imshow('frame',frame)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            sys.exit()

    def callback(self, data, id):
        if id==0:
            self.face_uv = data.data
            self.u = self.face_uv[0]
            self.v = self.face_uv[1]
            self.sync_flag = 1
            # print (self.face_uv)

        if id==1 and self.sync_flag == 1:
            self.sync_flag = 0
            try:
                depth_image = self.bridge.imgmsg_to_cv2(data, 'passthrough')
            except CvBridgeError, e:
                rospy.logerr(e)
            self.uv_to_xyz(depth_image, self.u, self.v)
 
if __name__ == "__main__":
    rospy.init_node('face_recog')
    FaceRecog()
    rospy.spin()