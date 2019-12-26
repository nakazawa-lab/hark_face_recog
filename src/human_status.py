#!/usr/bin/env python
## coding: UTF-8

class Human_data:
    def __init__(self, human_id):
        # 人間のid
        self.human_id = human_id
        # 人間の位置
        self.xmin = None
        self.ymin = None
        self.xmax = None
        self.xmax = None
        # 人間の鼻の位置
        self.nose_x = None
        self.nose_y = None
        # 顔認識を行う長方形領域の位置
        self.upper = None
        self.lower = None
        self.left = None
        self.right = None
        # 人間のMouth Aspect Ratio
        self.mar = 0
        self.last_mar = 1 #　1フレーム前のMouth Aspect Ratio
        # 人間が話しているかどうかを判定する際に用いる
        self.mouth_count = 0
        self.speaking_flag = 0
        self.is_open_flag = 0
        # HARKに送信するデータ
        self.send_x = 0
        self.send_y = 0
        self.send_z = 0
        self.talk_id = 10000 + 1000 * human_id

