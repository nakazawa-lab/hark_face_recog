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
        # 人物のID判定用
        self.past_point = None # 前のフレームの顔(鼻)の位置を保持するために用意
        self.id = 10000 # 人物判定用のid
        self.rerecog_flag = 0 # 人の顔が画面から外れたあとに、再び画面に写った際にidを変えるために用意
        # 口が動いているかの判定用
        self.past_mouth_distance = None # 前のフレームの口の開き具合を保持するために用意
        self.mouth_count = 0 # 口の形状がどれくらいの時間維持されているかをカウントするために用意
        self.start_flag = 0 # 口の動きを判定し始める際の合図
        self.speaking_flag = 0 # 話している間１にし、話していないときは0にする


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

    # １つ前のフレームの鼻の位置と現在のフレームの鼻の位置から同一人物を判定
    def human_identify(self, x, y, flag):
        if flag == 0:
            if self.past_point == None:
                self.past_point = (x, y)
                self.id = 10001
            else:
                past_x = self.past_point[0]
                past_y = self.past_point[1]

                if past_x - 30 < x and x < past_x + 30 and past_y - 30 < y and y < past_y + 30:
                    print("同一人物")
                else:
                    print("別人")
                    self.id += 1
                self.past_point = (x, y)
        else:
            self.id += 1
            self.rerecog_flag = 0
        print(self.id)
        return self.id

    # 口が動いているかを判定(口が動いているときはTrueを返し、口が動いていないときはFalseを返す)
    def mouth_motion(self, mouth_upper, mouth_lower, flag):

        print("mouth_distance:", self.past_mouth_distance)
        print("mouth_count:", self.mouth_count)
        now_mouth_distance = mouth_lower - mouth_upper
        if self.past_mouth_distance == None:
            pass
        else:
            # 口の開き具合が1フレーム前の開き具合と同じ場合カウントを1ずつ増やしていく
            if self.past_mouth_distance-2 <= now_mouth_distance and now_mouth_distance <= self.past_mouth_distance+2:
                self.mouth_count += 1
            # 口の開き具合が１フレーム前の開き具合と異なる場合カウントを0にする
            else:
                self.mouth_count = 0

        self.past_mouth_distance = now_mouth_distance

        # カウントが4以上の場合人が話していないと判断する
        if self.mouth_count >= 4:
            print("話していません")
            self.start_flag = 1 # 1度カウントが4を超えたらフラグを立てて(1にして)、以後はカウントが4より小さい場合に口が動いていると判定する
            return False
        else:
            if self.start_flag == 1:
                print("話しています")
                if self.speaking_flag == 0:
                    self.id += 1
                    self.speaking_flag = 1

                return True
            else:
                print("話していません")
                return False


    def callback(self, data):
        cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
        cv_image = imutils.resize(cv_image, self.width)

        # デバッグ用表示
        display_image = self.face.face_shape_detector_display(cv_image)
        cv2.imshow('img', display_image)
        cv2.waitKey(1)

        # face_recog_result = self.face.get_mouth_xy(cv_image)

        face_recog_result = self.face.get_nose_xy(cv_image)

        # 顔認識結果がある場合
        if not face_recog_result == None:
            u = face_recog_result[0]
            v = face_recog_result[1]
            theta = self.acquire_face_angle(u)
            print("face angle: " + str(-theta) + " [degree]")

            x = -(u - self.width/2)
            y = (v - self.height/2)
            z = self.f
            id = self.human_identify(x, y, self.rerecog_flag)

            # 口の座標データを取得
            mouth_recog_result = self.face.get_mouth_xy(cv_image)
            # 口の上部のy座標を算出
            v_upper = mouth_recog_result[1]
            y_upper = (v_upper - self.height/2)
            # 口の下部のy座標を算出
            v_lower = mouth_recog_result[3]
            y_lower = (v_lower - self.height/2)

            # 口が動いていないときは顔認識結果がないと判定し、0を送る
            is_mouth_moving = self.mouth_motion(y_upper, y_lower, self.speaking_flag)
            print(is_mouth_moving)
            if not is_mouth_moving:
                x = 0
                y = 0
                z = 0
                id = 0
                self.speaking_flag = 0

            self.send_to_ROS(x, y, z, id)

        # 顔認識結果がない場合は0を送る
        else:
            x = 0
            y = 0
            z = 0
            id = 0
            self.rerecog_flag = 1

            self.send_to_ROS(x, y, z, id)

if __name__ == "__main__":
    rospy.init_node('face_recog_to_ROS',anonymous=True)
    face_recognition = SendFaceToROS()
    rospy.spin()
