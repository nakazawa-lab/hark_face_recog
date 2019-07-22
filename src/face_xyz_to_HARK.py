#!/usr/bin/env python
## coding: UTF-8

import roslib
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from hark_msgs.msg import HarkSource
from hark_msgs.msg import HarkSourceVal
from std_msgs.msg import Int64MultiArray

import numpy
import pylab
import time
import math

class SendFaceXYZToHark:

    def __init__(self):
        # メンバ変数
        self.face_xy = []
        self.sync_flag = 0
        # メソッド
        self._face_xyz_pub = rospy.Publisher('face_xyz', HarkSource, queue_size=10)
        self._face_recog_sub = rospy.Subscriber('/face_recog_result', Int64MultiArray, self.callback, callback_args=0)
        self._pcl_sub = rospy.Subscriber('/kinect2/sd/points', PointCloud2, self.callback, callback_args=1)

    def coordinate_publish(self, xyz_dict):

        pub = rospy.Publisher('HarkSource', HarkSource, queue_size=10) # chatterという名前のTopicを生成し型やらを定義
        rate = rospy.Rate(10) # 10Hzで動かすrateというクラスを生成
        print("Conection started...")
        while not rospy.is_shutdown():

            data_to_HARK = []
            header = {}
            src = {}
            count = 0
            exist_src_num = 0

            #print("x", xyz_dict["x"], "y", xyz_dict["y"], "z", xyz_dict["z"])
            print(xyz_dict)


            """
            args should be ['header', 'count', 'exist_src_num', 'src']
            """

            oHarkTime = rospy.Time.now()

            header["stamp"] = oHarkTime
            header["frame_id"] = "HarkRosFrameID"

            src["id"] = 0
            src["power"] = 38.5952
            src["x"] = xyz_dict["x"]
            src["y"] = xyz_dict["y"]
            src["z"] = xyz_dict["z"]
            src["azimuth"] = 180 / math.pi * math.atan2(xyz_dict["y"], xyz_dict["x"])
            src["elevation"] = 180 / math.pi * math.atan2(xyz_dict["z"], math.sqrt(xyz_dict["x"] * xyz_dict["x"] + xyz_dict["y"] * xyz_dict["y"]))

            data_to_HARK.append(header)
            data_to_HARK.append(count)
            data_to_HARK.append(exist_src_num)
            data_to_HARK.append(src)
            print(data_to_HARK)

            # send_list = []
            # src = []
            # header = None
            # count = 0
            # exis_src_num = 0
            # send_list.append(header)
            # send_list.append(count)
            # send_list.append(exist_src_num)
            # send_list.append(src)


            pub.publish(data_to_HARK) # HARKへ送信
            #pub.publish(send_list)
            rate.sleep() # 先程定義したrateをここで動かす

    def callback(self, data, id):
        if id==0:
            self.face_xy = data.data
            self.sync_flag = 1
            # print (data)

        if id==1 and self.sync_flag == 1:
            self.sync_flag = 0
            resolution = (data.height, data.width)
            print("resolution", resolution)

            # 3D position for each pixel
            img = numpy.fromstring(data.data, numpy.float32)

            clp0 = []
            clp1 = []
            clp2 = []
            for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=False):
                clp0.append(p[0])
                clp1.append(p[1])
                clp2.append(p[2])

            x_points = numpy.array(clp0, dtype=numpy.float32)
            y_points = numpy.array(clp1, dtype=numpy.float32)
            z_points = numpy.array(clp2, dtype=numpy.float32)

            x = x_points.reshape(resolution)
            y = y_points.reshape(resolution)
            z = z_points.reshape(resolution)

            # print ("x0: " + str(x[self.face_xy[0] , self.face_xy[1]]) + ", y0: " + str(y[self.face_xy[0] , self.face_xy[1]]) + ", z0: " + str(z[self.face_xy[0] , self.face_xy[1]]))

            #取得したx,y,z座標をディクショナリに格納してHARKへ送る
            xyz_dict = {}
            xyz_dict["x"] = x[self.face_xy[0] , self.face_xy[1]]
            xyz_dict["y"] = y[self.face_xy[0] , self.face_xy[1]]
            xyz_dict["z"] = z[self.face_xy[0] , self.face_xy[1]]

            self.coordinate_publish(xyz_dict)


if __name__ == "__main__":
    rospy.init_node('face_xyz_to_HARK', anonymous=True)
    FaceXYZ = SendFaceXYZToHark()
    rospy.spin()
