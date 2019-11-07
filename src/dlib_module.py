#! /usr/bin/python
# -*- coding: utf-8 -*-

import argparse
import cv2
import numpy as np
import dlib
import sys
import datetime
import os
import imutils
import math
from imutils import face_utils
# from scipy.spatial import distance as dist # for desktop

class FaceDLib:

    def __init__(self, predictor_path):
        self.predictor = dlib.shape_predictor(predictor_path)

    def face_shape_detector_display(self, img, img_gray, rects, mar, MAR_THRESH):

        for rect in rects:
            # 画像の中から顔の特徴点を取得する
            shape = self.predictor(img_gray, rect)
            shape = face_utils.shape_to_np(shape)

            # MARの値を画面に表示する
            cv2.putText(img, "MAR: {:.2f}".format(mar), (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # 口が開いている場合、画面に表示する
            if mar > MAR_THRESH:
                cv2.putText(img, "Mouth is Open!", (30,60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)

            # cv2.rectangle(img, (rect.left(), rect.top()), (rect.right(), rect.bottom()), (255, 0, 0), 1)

            # landmarkを画像に書き込む 48:68が口
            mouth_hull = cv2.convexHull(shape[48:68])
            cv2.drawContours(img, [mouth_hull], -1, (0, 0, 255), 2)
            # for (x, y) in shape[48:68]:
            #     cv2.circle(img, (x, y), 5, (0, 0, 255), -1)

        return img

    """
    # 画像に映る口の座標を取得(口が動いているかを判別するために作成)→現在は使っていない
    def get_mouth_xy(self, img_gray):
        x_upper = x_lower = y_upper = y_lower = None # 初期化
        rects = self.detector(img_gray, 0)
        for rect in rects:
            shape = self.predictor(img_gray, rect)
            mouth_point_upper = shape.part(51) # 唇の上の点を取得
            x_upper = mouth_point_upper.x
            y_upper = mouth_point_upper.y
            mouth_point_lower = shape.part(57) # 唇の下の点を取得
            x_lower = mouth_point_lower.x
            y_lower = mouth_point_lower.y
        return x_upper, y_upper, x_lower, y_lower  # x, yはそれぞれlong型で、それらをタプルとして返す。
    """
    # mouth aspect ratio(口の開き具合の指標)を算出
    def mouth_aspect_ratio(self, img_gray, rects):
        mar = 0 # mouth aspect ratio初期化
        for rect in rects:
            shape = self.predictor(img_gray, rect)
            shape = face_utils.shape_to_np(shape)

            # 顔の各特徴点の座標を取得(Desktop)
            # A = dist.euclidean(shape[51] , shape[59]) # 51, 59
            # B = dist.euclidean(shape[53] , shape[57]) # 53, 57
            # C = dist.euclidean(shape[49] , shape[55]) # 49, 55

            # 顔の各特徴点の座標を取得(xavier)
            A = np.linalg.norm(shape[51] - shape[59])
            B = np.linalg.norm(shape[53] - shape[57])
            C = np.linalg.norm(shape[49] - shape[55])

        	# mouth aspect ratioの計算(口が開いているほど値が大きくなる)
            mar = (A + B) / (2.0 * C)

        return mar

    # 画像に映る鼻の座標を取得(IDを割り振る際の人物判定用に作成)
    def get_nose_xy(self, img_gray, rects):
        x = y = None # 初期化
        for rect in rects:
            shape = self.predictor(img_gray, rect)
            nose_point = shape.part(30) # 鼻の点を取得
            x = nose_point.x
            y = nose_point.y
        return x, y  # x, yはそれぞれlong型で、それらをタプルとして返す。


if __name__ == '__main__':
    predictor_path = "./shape_predictor_68_face_landmarks.dat"
    gets = FaceDLib(predictor_path)
    cap = cv2.VideoCapture(0)
    print("frame per second:", cap.get(cv2.CAP_PROP_FPS))
    count = 0

    while True:
        ret, frame = cap.read()
        frame = imutils.resize(frame, width=500)
        img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame, img_gray = gets.face_shape_detector_display(frame, img_gray)
        cv2.imshow('img', frame)
        c = cv2.waitKey(1)
        if c == 27:#ESCを押してウィンドウを閉じる
            break
        if c == 32:#spaceで保存
            count += 1
            cv2.imwrite('./filename%03.f'%(count)+'.jpg', frame) #001~連番で保存
            print('save done')
        gets.get_mouth_xy(img_gray)
        gets.get_nose_xy(img_gray)
        print("get_nose_xy: " + str(gets.get_nose_xy(img_gray)))
        print("get_mouth_xy: " + str(gets.get_mouth_xy(img_gray)))
    cap.release()
    cv2.destroyAllWindows()
