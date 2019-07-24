#!/usr/bin/env python
## coding: UTF-8

import roslib
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from hark_msgs.msg import HarkSource
from hark_msgs.msg import HarkSourceVal
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Float32MultiArray


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
        # self._face_xyz_pub = rospy.Publisher('face_xyz', HarkSource, queue_size=10)
        self._face_recog_sub = rospy.Subscriber('/face_recog_result', Int64MultiArray, self.callback, callback_args=0)
        self._pcl_sub = rospy.Subscriber('/kinect2/sd/points', PointCloud2, self.callback, callback_args=1)

    def coordinate_publish(self, xyz_dict):

        pub = rospy.Publisher('face_xyz', Float32MultiArray, queue_size=10)
        rate = rospy.Rate(10) # 10Hzで動かすrateというクラスを生成
        print("Conection started...")

        print(xyz_dict)

        """
        args should be ['header', 'count', 'exist_src_num', 'src']
        """

        xyz_array = Float32MultiArray(data=[xyz_dict["x"], xyz_dict["y"], xyz_dict["z"]])

        pub.publish(xyz_array) # face_xyz_to_HARK.cppへ送信
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
            #print("resolution", resolution)

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

            #取得したx,y,z座標をディクショナリに格納してface_xyz_to_HARK.cppへ送る
            xyz_dict = {}
            xyz_dict["x"] = x[self.face_xy[0] , self.face_xy[1]]
            xyz_dict["y"] = y[self.face_xy[0] , self.face_xy[1]]
            xyz_dict["z"] = z[self.face_xy[0] , self.face_xy[1]]

            self.coordinate_publish(xyz_dict)


if __name__ == "__main__":
    rospy.init_node('face_xyz_to_HARK_py', anonymous=True)
    FaceXYZ = SendFaceXYZToHark()
    rospy.spin()
