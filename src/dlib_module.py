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
from imutils import face_utils

class FaceDLib:

    def __init__(self, predictor_path):
        self.predictor = dlib.shape_predictor(predictor_path)
        self.detector = dlib.get_frontal_face_detector()

    def face_shape_detector_display(self, img):
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # The 1 in the second argument indicates that we should upsample the image
        # 1 time.  This will make everything bigger and allow us to detect more
        # faces.
        # Finally, if you really want to you can ask the detector to tell you the score
        # for each detection.  The score is bigger for more confident detections.
        # The third argument to run is an optional adjustment to the detection threshold,
        # where a negative value will return more detections and a positive value fewer.
        # Also, the idx tells you which of the face sub-detectors matched.  This can be
        # used to broadly identify faces in different orientations.
        dets, scores, idx = self.detector.run(img_rgb, 1, 0)
        # print "dlib score: " + str(scores)
        # print "dlib idx: " + str(idx)
        if len(dets) > 0:
            for i, rect in enumerate(dets):
                # print("Detection {}: Left: {} Top: {} Right: {} Bottom: {}".format(i, rect.left(), rect.top(), rect.right(), rect.bottom()))
                shape = self.predictor(img_rgb, rect)
                shape = face_utils.shape_to_np(shape)
                clone = img.copy()
                cv2.rectangle(clone, (rect.left(), rect.top()), (rect.right(), rect.bottom()), (255, 0, 0), 1)
                # cv2.putText(clone, "dlibScore: " + str(scores), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                # landmarkを画像に書き込む 48:68が口。
                for (x, y) in shape[0:68]:
                    cv2.circle(clone, (x, y), 5, (0, 0, 255), -1)
            return clone
        else :
            return img

    def get_mouth_xy(self, img):
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        dets, scores, idx = self.detector.run(img_rgb, 1, 0)
        if len(dets) > 0:
            for i, rect in enumerate(dets):
                shape = self.predictor(img_rgb, rect)
                mouth_point = shape.part(63) # 唇の上の点を取得
                x = mouth_point.x
                y = mouth_point.y
            return x, y  # x, yはそれぞれlong型で、それらをタプルとして返す。
        else :
            return None

    # IDを割り振る際の人物判定用に作成
    def get_nose_xy(self, img):
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        dets, scores, idx = self.detector.run(img_rgb, 1, 0)
        if len(dets) > 0:
            for i, rect in enumerate(dets):
                shape = self.predictor(img_rgb, rect)
                mouth_point = shape.part(30) # 鼻の点を取得
                x = mouth_point.x
                y = mouth_point.y
            return x, y  # x, yはそれぞれlong型で、それらをタプルとして返す。
        else :
            return None

if __name__ == '__main__':
    predictor_path = "./shape_predictor_68_face_landmarks.dat"
    gets = FaceDLib(predictor_path)
    cap = cv2.VideoCapture(0)
    count = 0

    while True:
        ret, frame = cap.read()
        frame = imutils.resize(frame, width=500)
        frame  = gets.face_shape_detector_display(frame)
        cv2.imshow('img', frame)
        c = cv2.waitKey(1)
        if c == 27:#ESCを押してウィンドウを閉じる
            break
        if c == 32:#spaceで保存
            count += 1
            cv2.imwrite('./filename%03.f'%(count)+'.jpg', frame) #001~連番で保存
            print('save done')
        gets.get_mouth_xy(frame)
        gets.get_nose_xy(frame)
        print("get_nose_xy: " + str(gets.get_nose_xy(frame)))
        print("get_mouth_xy: " + str(gets.get_mouth_xy(frame)))
    cap.release()
    cv2.destroyAllWindows()
