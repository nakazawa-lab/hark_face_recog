#!/usr/bin/env python
## coding: UTF-8

# ros系のライブラリ
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
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
import argparse

# 自作モジュール
import dlib_module as dm

class SendFaceToROS:
    def __init__(self):
        # 顔認識に関する記述
        here_path = os.path.dirname(__file__)
        if here_path == "":
            here_path = "."
        self.predictor_path = here_path + "/shape_predictor_68_face_landmarks.dat"
        #self.predictor_path = "./shape_predictor_68_face_landmarks.dat"
        self.face = dm.FaceDLib(self.predictor_path)
        # ROSのメソッド
        self._face_recog_pub = rospy.Publisher('face_recog_result', Float32MultiArray, queue_size=10)
        # self._image_sub = rospy.Subscriber('/kinect2/hd/image_color', Image, self.callback)
        # OpenCVのメソッド
        self._bridge = CvBridge()
        # dlibのメソッド
        self.detector = dlib.get_frontal_face_detector()
        # kinectカメラの情報を取得
        # camera_info = rospy.wait_for_message("/kinect2/qhd/camera_info", CameraInfo)
        # self.width = int(camera_info.width * 1.2)
        # self.height = int(camera_info.height * 1.2)
        self.width = int(1152)
        self.height = int(648)
        # camera_info = rospy.wait_for_message("/kinect2/hd/camera_info", CameraInfo)
        # self.width = camera_info.width
        # self.height = camera_info.height
        # K = np.array(camera_info.K).reshape(3,3) # 参照：http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        # self.f = K[0][0] # 焦点距離f
        self.f = 540.68603515625  # 焦点距離f
        # 人物のID判定用
        self.past_point = None # 前のフレームの顔(鼻)の位置を保持するために用意
        self.id = 10000 # 人物判定用のid
        self.rerecog_flag = 0 # 人の顔が画面から外れたあとに、再び画面に写った際にidを変えるために用意
        # 口が動いているかの判定用
        # self.past_mouth_distance = None # 前のフレームの口の開き具合を保持するために用意
        # self.mouth_count = 0 # 口の形状がどれくらいの時間維持されているかをカウントするために用意
        self.mouth_close_count = 0 # 口がどれくらいの時間閉まっているかをカウントするために用意
        self.MAR_THRESH = 0.70 # mouth aspect ratioの閾値(marの値がこの値以上になった場合口が開いていると判断する)
        self.start_flag = 0 # 口の動きを判定し始める際の合図
        self.speaking_flag = 0 # 話している間１にし、話していないときは0にする

        # 動画の読み込み開始サインをHARK側から受け取る
        self.movie_sign = rospy.wait_for_message("start_sign", String)

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

if __name__ == "__main__":

    #　コマンドライン引数を設定
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--movie_path', default='/home/nvidia/catkin_ws/src/hark_face_recog/src/userdata/records/movie_dataset/dataset2/movie_data.mp4', help='読み込みたい動画データのパス')
    args = parser.parse_args()

    # ROSの初期化
    rospy.init_node('face_recog_to_ROS', anonymous=True)

    # インスタンス生成
    send_ros = SendFaceToROS()
    face_data = dm.FaceDLib(send_ros.predictor_path)

    # 動画データの読み込み
    cap = cv2.VideoCapture(args.movie_path)
    print("frame per second:", cap.get(cv2.CAP_PROP_FPS))
    count = 0

    # 動画が終わるまで処理を続ける
    while cap.isOpened():
        ret, cv_img = cap.read()
        cv_img = imutils.resize(cv_img, send_ros.width)
        cv_img = cv_img[0:send_ros.height / 2]

        # グレースケールの画像を取得
        img_gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        # 画像の中から顔を検出
        rects = send_ros.detector(img_gray, 0)

        # 鼻の座標データを取得
        face_recog_result = face_data.get_nose_xy(img_gray, rects)

        # mouth aspect ratio(口の開き具合の指標)を取得
        mar = send_ros.face.mouth_aspect_ratio(img_gray, rects)

        # 顔認識結果がある場合
        if not face_recog_result[0] == None:
            u = face_recog_result[0]
            v = face_recog_result[1]
            theta = send_ros.acquire_face_angle(u)
            print("face angle: " + str(-theta) + " [degree]")

            x = -(u - send_ros.width / 2)
            y = (v - send_ros.height / 2)
            z = send_ros.f
            id = send_ros.id

            # 口が動いていないときは顔認識結果がないと判定し、0を送る
            is_mouth_moving = send_ros.mouth_motion_with_mar(mar, send_ros.speaking_flag)

            if not is_mouth_moving:
                x = 0
                y = 0
                z = 0
                id = 0
                # 話し終わったタイミングでidを1増やす
                if send_ros.speaking_flag == 1:
                    send_ros.id += 1
                send_ros.speaking_flag = 0
            send_ros.send_to_ROS(x, y, z, id)

        # 顔認識結果がない場合は0を送る
        else:
            x = 0
            y = 0
            z = 0
            id = 0
            send_ros.mouth_close_count += 1
            send_ros.rerecog_flag = 1
            send_ros.send_to_ROS(x, y, z, id)

        # デバッグ用表示
        display_image = face_data.face_shape_detector_display(cv_img, img_gray, rects, mar, send_ros.MAR_THRESH)

        cv2.imshow('img', display_image)
        cv2.waitKey(1)

        # c = cv2.waitKey(1)
        # if c == 27:  # ESCを押してウィンドウを閉じる
        #     break
        # if c == 32:  # spaceで保存
        #     count += 1
        #     cv2.imwrite('./filename%03.f' % (count) + '.jpg', cv_img)  # 001~連番で保存
        #     print('save done')
        # send_ros.face.get_mouth_xy(img_gray)
        # send_ros.face.get_nose_xy(img_gray)
        # print("get_nose_xy: " + str(send_ros.face.get_nose_xy(img_gray)))
        # print("get_mouth_xy: " + str(send_ros.face.get_mouth_xy(img_gray)))

    cap.release()
    cv2.destroyAllWindows()

