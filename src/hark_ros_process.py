from harkpython import harkbasenode
import math

class HarkNode(harkbasenode.HarkBaseNode):
    def __init__(self):
        print("=" * 30 )
        self.outputNames=("OUTPUT",)  # one output terminal named "output"
        #self.outputTypes=("prim_float",)  # the type is primitive float.

        self.outputTypes=("vector_source",)

    def calculate(self):

        # デバック用
        if len(self.SOURCES)<1:
            print("sound_source: no data")
        else:
            #print(self.SOURCES[0])
            print("sound_source:", self.SOURCES)


        if len(self.SOURCES2)<1:
            print("image_source: no data")
        else:
            #print(self.SOURCES[0])
            print("image_source:", self.SOURCES2)


        # 顔の方位角方向と音源の方位角が一致した場合にその方向の音のみを分離する
        # 今はテスト用に顔の方位角と音源の方位角が一致した場合にのみその方向の音を分離するプログラムにしているが、
        # いずれは顔の方位角が検出できない場合でも、音源の方位角のみで分離できるようにする
        if len(self.SOURCES)<1 or len(self.SOURCES2)<1:
            print("音源方向または顔方向のデータがありません")
            self.outputValues["OUTPUT"] = []　

        else:
            # 音源の位置座標を取得
            sound_xyz_coordinate = self.SOURCES[0]['x']

            sound_x = sound_xyz_coordinate[0]
            sound_y = sound_xyz_coordinate[1]
            sound_z = sound_xyz_coordinate[2]
            sound_azimuth = 180.0 / math.pi * math.atan2(sound_y, sound_x); # 音源の方位角(°)を計算

            # 顔の位置座標を取得
            img_xyz_coordinate = self.SOURCES2[0]['x']

            img_x = img_xyz_coordinate[0]
            img_y = img_xyz_coordinate[1]
            img_z = img_xyz_coordinate[2]
            img_azimuth = 180.0 / math.pi * math.atan2(img_y, img_x); # 顔の方位角(°)を計算

            # 認識した顔の方位角±5°の方向から到来した音のみを分離する
            if abs(sound_azimuth - img_azimuth) < 5.00:
                self.outputValues["OUTPUT"] = [self.SOURCES[0]]

            else:
                print("音源方向と顔方向が一致していません")
                self.outputValues["OUTPUT"] = []

        # 出力の値を設定
        #self.outputValues["OUTPUT"] = [self.SOURCE[0]]
        #self.outputValues["OUTPUT"] = [] if len(self.SOURCES)<1 else [self.SOURCES[0]]
        # self.outputValues["OUTPUT"] = [] if len(self.SOURCES2)<1 else [self.SOURCES2[0]]


        #定数入力は下記のように行う
        #self.outputValues["OUTPUT"] = [{'x': [-0.9433000087738037, -0.1662999987602234, 0.2874000072479248], 'power': 34.90971374511719, 'id': 2}]
