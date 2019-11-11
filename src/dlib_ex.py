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

# 自作モジュール
import dlib_module as dm

class DlibEx:
    def __init__(self):
        # 顔認識に関する記述
        here_path = os.path.dirname(__file__)
        if here_path == "":
            here_path = "."
        self.predictor_path = here_path + "/shape_predictor_68_face_landmarks.dat"
        self.face = dm.FaceDLib(self.predictor_path)
        # ROSのメソッド
        self._image_sub = rospy.Subscriber('dsampled_image', Image, self.callback)
        # OpenCVのメソッド
        self._bridge = CvBridge()
        # dlibのメソッド
        self.detector = dlib.get_frontal_face_detector()
        # kinectカメラの情報を取得
        camera_info = rospy.wait_for_message("/kinect2/qhd/camera_info", CameraInfo)
        self.width = int(camera_info.width * 1.2)
        self.height = int(camera_info.height * 1.2)
        # 録画用の設定
        fourcc = cv2.VideoWriter_fourcc(*"DIVX")
        save_path = here_path + "/userdata/"
        self.writer = cv2.VideoWriter("output.avi", fourcc, fps, (width, height))

        # # 人物のID判定用
        # self.past_point = None # 前のフレームの顔(鼻)の位置を保持するために用意
        # self.id = 10000 # 人物判定用のid
        # self.rerecog_flag = 0 # 人の顔が画面から外れたあとに、再び画面に写った際にidを変えるために用意
        # self.mouth_close_count = 0 # 口がどれくらいの時間閉まっているかをカウントするために用意
        # self.MAR_THRESH = 0.70 # mouth aspect ratioの閾値(marの値がこの値以上になった場合口が開いていると判断する)
        # self.start_flag = 0 # 口の動きを判定し始める際の合図
        # self.speaking_flag = 0 # 話している間１にし、話していないときは0にする

    def send_to_ROS(self, x, y, z, id):
        array = Float32MultiArray(data=[x, y, z, id])
        self._face_recog_pub.publish(array)

    def acquire_face_angle(self, u):
        radian = math.atan((u - self.width/2) / self.f)
        theta = np.rad2deg(radian)
        return theta

    def theta_debug(self, theta):
        radian = np.radians(theta)
        u = math.tan(radian) * self.f + self.width/2
        return u

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

    # ROSによって繰り返し呼び出される
    def callback(self, data):
        if rospy.is_shutdown():
            self.writer.release()

        cv_img = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        cv_img = imutils.resize(cv_img, self.width)
        cv_img = cv_img[0:self.height/2]

        # グレースケールの画像を取得
        img_gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        # 画像の中から顔を検出
        # rects, scores, idx  = self.detector.run(img_gray, 0, 0)
        rects = self.detector(img_gray, 0)
        # print ("score" + str(scores))

        self.writer.write(frame)
        #
        # # mouth aspect ratio(口の開き具合の指標)を取得
        # mar = self.face.mouth_aspect_ratio(img_gray, rects)
        #
        # # 顔認識結果がある場合
        # if not face_recog_result[0] == None:
        #     u = face_recog_result[0]
        #     v = face_recog_result[1]
        #     theta = self.acquire_face_angle(u)
        #     print("face angle: " + str(-theta) + " [degree]")
        #
        #     x = -(u - self.width/2)
        #     y = (v - self.height/2)
        #     z = self.f
        #     # id = self.human_identify(x, y, self.rerecog_flag)
        #     id = self.id
        #
        #     # 口が動いていないときは顔認識結果がないと判定し、0を送る
        #     is_mouth_moving = self.mouth_motion_with_mar(mar, self.speaking_flag)
        #
        #     if not is_mouth_moving:
        #         x = 0
        #         y = 0
        #         z = 0
        #         id = 0
        #         # 話し終わったタイミングでidを1増やす
        #         if self.speaking_flag == 1:
        #             self.id += 1
        #         self.speaking_flag = 0
        #     self.send_to_ROS(x, y, z, id)
        #
        # # 顔認識結果がない場合は0を送る
        # else:
        #     x = 0
        #     y = 0
        #     z = 0
        #     id = 0
        #     self.mouth_close_count += 1
        #     self.rerecog_flag = 1
        #     self.send_to_ROS(x, y, z, id)
        #
        # # デバッグ用表示
        # display_image = self.face.face_shape_detector_display(cv_img, img_gray, rects,  mar, self.MAR_THRESH)
        #
        # cv2.imshow('img', display_image)
        # cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('dlib_ex',anonymous=True)
    face_recognition = DlibEx()
    rospy.spin()
