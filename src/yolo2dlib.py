#!/usr/bin/env python
## coding: UTF-8

# ros系のライブラリ
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
import datetime
from PIL import Image as pil
import copy

# ros以外
import cv2
import numpy as np
import math
import dlib
from cv_bridge import CvBridge, CvBridgeError
from imutils import face_utils
import os

# 自作モジュール
import dlib_module as dm
from human_status import Human_data

class YOLO2Dlib:
    def __init__(self):
        # camera_info = rospy.wait_for_message("/usb_cam/camera_info", CameraInfo)
        # camera_info = rospy.wait_for_message("/kinect2/hd/camera_info", CameraInfo)
        self.set_camera_info()

        self.person_bboxes = []
        self.padding = 10
        self.c = 0
        self.debug_image = np.zeros((800, 800, 3))
        self.here_path = os.path.dirname(__file__)
        if self.here_path == "":
            self.here_path = "."
        predictor_path = self.here_path + "/shape_predictor_68_face_landmarks.dat"
        self.face = dm.FaceDLib(predictor_path)
        self.predictor = dlib.shape_predictor(predictor_path)
        self.detector = dlib.get_frontal_face_detector()
        self.MAR_THRESH = 0.08
        # self.talk_id = 10000 # 会話のid
        # self.speaking_flag = 0 # 話しているときは1にし、話していないときは0にする
        # self.mouth_close_count = 0  # 口がどれくらいの時間閉まっているかをカウントするために用意
        # self.last_mar = 1
        self.frame = 0
        self.MOUTH_OPEN_DURATION_THRESH = 20
        # self.mouth_count = self.MOUTH_OPEN_DURATION_THRESH * (-1)
        # self.is_open_flag = False
        # self.last_is_open_flag = False

        self.human_dict = {} # それぞれの人に対して生成したインスタンスを格納するディクショナリ
        self.hark_send_list = [] # HARKに送信するためのデータを格納するリスト, self.human_data_listを格納する

        self._bridge = CvBridge()
        self._usb_image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.usb_callback)
        self._kinect_image_sub = rospy.Subscriber('/kinect2/hd/image_color', Image, self.kinect_callback)
        self._dataset_image_sub = rospy.Subscriber('/moviedata_py', Image, self.dataset_callback)
        self._bboxes_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bboxes_callback)
        self._face_recog_pub = rospy.Publisher('face_recog_result', Float32MultiArray, queue_size=10)
        self.m_pub_threshold = rospy.get_param('~pub_threshold', 0.750)  # ROS PARAM

        return

    def set_camera_info(self):
        self.width = 1920
        self.height = 1080
        self.f = 1081.3720703125
    # def set_camera_info(self, camera_info):
    #     self.width = camera_info.width
    #     self.height = camera_info.height
    #     K = np.array(camera_info.K).reshape(3,3) # 参照：http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
    #     self.f = K[0][0] # 焦点距離f

    def usb_callback(self, data):
        try:
            image = self._bridge.imgmsg_to_cv2(data, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)
        self.image_callback(image, "usb")

    def kinect_callback(self, data):
        try:
            image = self._bridge.imgmsg_to_cv2(data, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)
        self.image_callback(image, "kinect")

    def dataset_callback(self, data):
        try:
            image = self._bridge.imgmsg_to_cv2(data, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)
        self.image_callback(image, "kinect")

    def bboxes_callback(self, msgs):
        bboxes = msgs.bounding_boxes
        person_bboxes = []
        if len(bboxes) != 0 :
            for i, bb in enumerate(bboxes):
                if bboxes[i].Class == 'person' and bboxes[i].probability >= self.m_pub_threshold:
                    person_bboxes.append(bboxes[i])
        self.person_bboxes = person_bboxes

    def image_callback(self, img, mode):
        debug_img = img
        # personを認識した場合
        if len(self.person_bboxes) != 0:
            for human_id, pbox in enumerate(self.person_bboxes):
                human_data_list = []  # 人の位置データとtalk IDを格納するリスト
                # 複数人の開口判定・閉口判定を同時に行うために画面に映る人それぞれに対してインスタンスを生成し、ディクショナリに格納する。
                # インスタンスがディクショナリにすでに含まれている場合は生成しない。
                # if len(self.human_dict) < len(self.person_bboxes):
                if len(self.human_dict) < len(self.person_bboxes) and human_id not in self.human_dict:
                    human_data = Human_data(human_id)
                    self.human_dict[human_id] = human_data
                # 画像から人の顔の部分を切り出し、顔認識
                debug_img = self.yolo_display(debug_img, pbox)
                crop = self.bbox2image(debug_img, pbox, human_id)
                if len(crop) != 0:
                    img_gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
                    dlib_rects = self.detector(img_gray, 0)
                    # もしdlibの顔認識に成功したら
                    if len(dlib_rects) != 0:
                        self.human_dict[human_id].mar = self.face.mouth_aspect_ratio(img_gray, dlib_rects)
                        # debug_img = self.dlib_display(debug_img, img_gray, dlib_rects, self.human_dict[human_id].mar, self.MAR_THRESH, human_id)
                        debug_img = self.dlib_display(debug_img, img_gray, dlib_rects, human_id)
                        self.human_dict[human_id].last_mar = self.human_dict[human_id].mar
                        # 口が動いているときは顔の定位情報をHARKに送る, 口が動いていないときは0をHARKに送る
                        if self.human_dict[human_id].is_open_flag == True:
                            self.human_dict[human_id].send_x = -(self.human_dict[human_id].nose_x - self.width/2)
                            self.human_dict[human_id].send_y = (self.human_dict[human_id].nose_y - self.height/2)
                            self.human_dict[human_id].send_z = self.f
                        else:
                            self.human_dict[human_id].send_x = 0
                            self.human_dict[human_id].send_y = 0
                            self.human_dict[human_id].send_z = 0
                        # HARKに送信するために各話者のデータをリストに格納していく
                        human_data_list.append(self.human_dict[human_id].send_x)
                        human_data_list.append(self.human_dict[human_id].send_y)
                        human_data_list.append(self.human_dict[human_id].send_z)
                        human_data_list.append(self.human_dict[human_id].talk_id)
                        # 各話者のデータを格納したリストをinterface_with_hark.cppに送信するリストに格納
                        self.hark_send_list.append(human_data_list)
        print(self.hark_send_list)
        self.send_to_ROS(self.hark_send_list)
        del self.hark_send_list[:] # リストの中身を空にする
        if mode == "usb":
            debug_img = cv2.cvtColor(debug_img, cv2.COLOR_RGBA2BGR)
        debug_img = cv2.resize(debug_img, dsize=None, fx=0.5, fy=0.5)
        cv2.namedWindow("image")
        cv2.imshow("image", debug_img)
        cv2.waitKey(1)
        self.frame = self.frame + 1

    def dlib_display(self, img, img_gray, rects, human_id):
        for rect in rects:
            # 画像の中から顔の特徴点を取得する
            shape = self.predictor(img_gray, rect)
            shape = face_utils.shape_to_np(shape)
            for p in shape[48:68]:
                p[0] = p[0] + self.human_dict[human_id].left
                p[1] = p[1] + self.human_dict[human_id].upper
            front_ratio = 100 - 100 * abs((shape[16][0]- shape[27][0]) - (shape[27][0]-shape[0][0])) / (shape[16][0]-shape[0][0])
            cv2.putText(img, "front: " + str(front_ratio) + "%", (1500, 1000), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 4)
            # 鼻に関して記述
            nose_p = shape[30]
            self.human_dict[human_id].nose_x = nose_p[0] + self.human_dict[human_id].left
            self.human_dict[human_id].nose_y = nose_p[1] + self.human_dict[human_id].upper
            cv2.circle(img, (self.human_dict[human_id].nose_x, self.human_dict[human_id].nose_y), 3, (0,0,255), thickness=5, lineType=cv2.LINE_8, shift=0)
            dmar = self.human_dict[human_id].mar - self.human_dict[human_id].last_mar
            # MARの値を画面に表示する
            cv2.putText(img, "human ID: {}, delta MAR: {:.2f}".format(human_id, dmar), (30, (human_id+1)*100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
            # 口が開いている場合、画面に表示する
            if dmar > self.MAR_THRESH:
                self.human_dict[human_id].mouth_count = self.frame
                self.human_dict[human_id].speaking_flag = True
            if self.frame - self.human_dict[human_id].mouth_count < self.MOUTH_OPEN_DURATION_THRESH:
                cv2.putText(img, "Mouth is Open!", (1100, (human_id+1)*100),
                            cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
                self.human_dict[human_id].is_open_flag = True
            else:
                self.human_dict[human_id].is_open_flag = False
            # 口が動いている状態から口が動かない状態になったら、talk_idを1増やす
            if self.human_dict[human_id].speaking_flag == True and self.human_dict[human_id].is_open_flag == False:
                self.human_dict[human_id].talk_id += 1
                self.human_dict[human_id].speaking_flag = False
                print("talk finish!")
            # landmarkを画像に書き込む 48:68が口
            mouth_hull = cv2.convexHull(shape[48:68])
            cv2.drawContours(img, [mouth_hull], -1, (0, 0, 255), 2)
        return img

    def yolo_display(self, image, pbox):
        cv2.rectangle(image, (pbox.xmin, pbox.ymin), (pbox.xmax, pbox.ymax),(0,0,255), 2)
        text = "score: " + str(round(pbox.probability, 3))
        text_top = (pbox.xmin, pbox.ymin + 40)
        text_bot = (pbox.xmin + 360, pbox.ymin + 5)
        text_pos = (pbox.xmin + 5, pbox.ymin+40)
        cv2.rectangle(image, text_top, text_bot, (0,0,0),-1)
        cv2.putText(image, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 255), 2)
        return image

    def bbox2image(self, image, pbox, human_id):
        crop = []
        if pbox.probability > 0.7:
            if pbox.ymin < self.padding:
                self.human_dict[human_id].upper = 0
            else:
                self.human_dict[human_id].upper = pbox.ymin - self.padding
            self.human_dict[human_id].lower = self.human_dict[human_id].upper + int((pbox.ymax - self.human_dict[human_id].upper) / 2.5)
            self.human_dict[human_id].left = pbox.xmin + (pbox.xmax - pbox.xmin) / 6
            self.human_dict[human_id].right = pbox.xmax - (pbox.xmax - pbox.xmin) / 6
            # dlibで認識できる最大横幅が72px
            if self.human_dict[human_id].right - self.human_dict[human_id].left > 72:
                crop = image[self.human_dict[human_id].upper:self.human_dict[human_id].lower, self.human_dict[human_id].left:self.human_dict[human_id].right]
                cv2.rectangle(image, (self.human_dict[human_id].right, self.human_dict[human_id].upper), (self.human_dict[human_id].left, self.human_dict[human_id].lower), (0, 0, 0), 2)
            else:
                crop = np.asarray(crop)
        else:
            crop = np.asarray(crop)
        return crop

    def send_to_ROS(self, send_list):
        # ROSで送信することができるように1次元データに変換する
        send_list_numpy = np.array(send_list) # リストをnumpy配列へ変換
        send_list_reshaped = send_list_numpy.flatten() # 2次元データを1次元データへ変換
        array = Float32MultiArray(data=send_list_reshaped)
        self._face_recog_pub.publish(array)

    def face_angle_debug(self, img, u):
        radian = math.atan((u - self.width/2) / self.f)
        theta = np.rad2deg(radian)
        cv2.putText(img, str(theta) + "°", (self.nose_x, self.nose_y + 1), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 0, 255), 1)
        return img

if __name__ == "__main__":
    rospy.init_node('yolo2dlib',anonymous=True)
    yd = YOLO2Dlib()
    rospy.loginfo('running..')
    rospy.spin()

