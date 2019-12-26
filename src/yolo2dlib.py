#!/usr/bin/env python
## coding: UTF-8

# ros系のライブラリ
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
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
        self.MAR_THRESH = 0.06
        self.id = 10000 # 人物判定用のid
        self.speaking_flag = 0 # 話している間１にし、話していないときは0にする
        # self.mouth_close_count = 0  # 口がどれくらいの時間閉まっているかをカウントするために用意
        # self.start_flag = 0  # 口の動きを判定し始める際の合図
        self.last_mar = 1
        self.frame = 0
        self.MOUTH_OPEN_DURATION_THRESH = 25
        self.mouth_count = self.MOUTH_OPEN_DURATION_THRESH * (-1)
        self.is_open_flag = False
        self.last_is_open_flag = False


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
        self.is_open_flag = False
        debug_img = img
        x = 0
        y = 0
        z = 0
        # id = 0
        # personを認識した場合
        if len(self.person_bboxes) != 0:
            for i, pbox in enumerate(self.person_bboxes):
                debug_img = self.yolo_display(debug_img, pbox)
                crop = self.bbox2image(debug_img, pbox)
                if len(crop) != 0:
                    img_gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
                    dlib_rects = self.detector(img_gray, 0)
                    # もしdlibの顔認識に成功したら
                    if len(dlib_rects) != 0:
                        mar = self.face.mouth_aspect_ratio(img_gray, dlib_rects)
                        debug_img = self.dlib_display(debug_img, img_gray, dlib_rects, mar, self.MAR_THRESH)
                        self.last_mar = mar
                        # if self.is_open_flag:
                    x = -(self.nose_x - self.width/2)
                    y = (self.nose_y - self.height/2)
                    z = self.f
                    id = self.id
        if self.last_is_open_flag and not self.is_open_flag:
            self.id += 1
        self.send_to_ROS(x, y, z, id)
        if mode == "usb":
            debug_img = cv2.cvtColor(debug_img, cv2.COLOR_RGBA2BGR)
        debug_img = cv2.resize(debug_img, dsize=None, fx=0.5, fy=0.5)
        cv2.namedWindow("image")
        cv2.imshow("image", debug_img)
        cv2.waitKey(1)
        self.last_is_open_flag = self.is_open_flag
        self.frame = self.frame + 1


    def dlib_display(self, img, img_gray, rects, mar, MAR_THRESH):
        for rect in rects:
            # 画像の中から顔の特徴点を取得する
            shape = self.predictor(img_gray, rect)
            shape = face_utils.shape_to_np(shape)
            for p in shape[48:68]:
                p[0] = p[0] + self.left
                p[1] = p[1] + self.upper
            front_ratio = 100 - 100 * abs((shape[16][0]- shape[27][0]) - (shape[27][0]-shape[0][0])) / (shape[16][0]-shape[0][0])
            cv2.putText(img, "front: " + str(front_ratio) + "%", (1500, 1000), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 4)
            # 鼻に関して記述
            nose_p = shape[30]
            self.nose_x= nose_p[0] + self.left
            self.nose_y = nose_p[1] + self.upper
            cv2.circle(img, (self.nose_x, self.nose_y), 3, (0,0,255), thickness=5, lineType=cv2.LINE_8, shift=0)
            dmar = mar - self.last_mar
            # MARの値を画面に表示する
            cv2.putText(img, "delta MAR: {:.2f}".format(dmar), (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
            # 口が開いている場合、画面に表示する
            if dmar > MAR_THRESH:
                self.mouth_count = self.frame
            if self.frame - self.mouth_count < self.MOUTH_OPEN_DURATION_THRESH:
                cv2.putText(img, "Mouth is Open!", (30,120),
                cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255),2)
                self.is_open_flag = True
            else:
                self.is_open_flag = False
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

    def bbox2image(self, image, pbox):
        crop = []
        if pbox.probability > 0.7:
            if pbox.ymin < self.padding:
                self.upper = 0
            else:
                self.upper = pbox.ymin - self.padding
            self.lower = self.upper + int((pbox.ymax - self.upper) / 2.5)
            self.left = pbox.xmin + (pbox.xmax - pbox.xmin) / 6
            self.right = pbox.xmax - (pbox.xmax - pbox.xmin) / 6
            # dlibで認識できる最大横幅が72px
            if self.right - self.left > 72:
                crop = image[self.upper:self.lower, self.left:self.right]
                cv2.rectangle(image, (self.right, self.upper), (self.left, self.lower), (0, 0, 0), 2)
            else:
                crop = np.asarray(crop)
        else:
            crop = np.asarray(crop)
        return crop

    # mouth_aspect_ratioを使用して口が動いているかを判定する
    # def mouth_motion_with_mar(self, mar, flag):
    #
    #     # 口が閉まっている場合カウントを1ずつ増やしていく
    #     if mar < self.MAR_THRESH:
    #         self.mouth_close_count += 1
    #     # 口が開いている場合にカウントを0にする
    #     else:
    #         self.mouth_close_count = 0
    #
    #     print("mouth_close_count:", self.mouth_close_count)
    #
    #     # カウントが10以上の場合人が話していないと判断する
    #     if self.mouth_close_count >= 10:
    #         self.start_flag = 1 # 1度カウントが10を超えたらフラグを立てて(1にして)、以後はカウントが10より小さい場合に口が動いていると判定する
    #         # print("話していません")
    #         return False
    #     else:
    #         if self.start_flag == 1:
    #             if self.speaking_flag == 0:
    #                 self.speaking_flag = 1
    #             # print("話しています")
    #             return True
    #         else:
    #             # print("話していません")
    #             return False

    def send_to_ROS(self, x, y, z, id):
        array = Float32MultiArray(data=[x, y, z, id])
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

