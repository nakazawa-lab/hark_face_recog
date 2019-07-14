#!/usr/bin/env python
## coding: UTF-8

import roslib
import rospy
from sensor_msgs.msg import PointCloud2
import numpy
import pylab
import time
import sensor_msgs.point_cloud2 as pc2

def callback(data):
    resolution = (data.height, data.width)

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

    print ("x: " + str(x[data.height/2 , data.width/4]) + ", y: " + str(y[data.height/2 , data.width/4]) + ", z: " + str(z[data.height/2 , data.width/4]))
    print ("x0: " + str(x[data.height/2 , data.width/2]) + ", y0: " + str(y[data.height/2 , data.width/2]) + ", z0: " + str(z[data.height/2 , data.width/2]))

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/kinect2/sd/points',
                     PointCloud2, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()
    ## テスト