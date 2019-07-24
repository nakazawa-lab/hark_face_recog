#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import message_filters
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
 
class DepthTeleopTurtle:
    def __init__(self):
        self.bridge = CvBridge()
         
        rgb_sub = message_filters.Subscriber('/kinect2/sd/image_color_rect', Image)
        depth_sub = message_filters.Subscriber('/kinect2/sd/image_depth_rect', Image)

        self.camera_info = rospy.wait_for_message("/kinect2/sd/camera_info", CameraInfo)
        self.invK = np.linalg.inv(np.array(self.camera_info.K).reshape(3,3))

        self.mf = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 30, 0.5)
        self.mf.registerCallback(self.callback)

    def callback(self, rgb_data , depth_data):
        info = self.camera_info
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_data, 'passthrough')
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)

        depth_array = np.array(depth_image, dtype=np.float32)

        image_point =np.array([info.height/2, info.width/2, 1])
        z = depth_array[info.height/2, info.height/2]
        xyz_point = np.matmul(self.invK, image_point) * z 
        
        x = round(xyz_point[0]/1000, 2)
        y = round(xyz_point[1]/1000, 2)
        z = round(xyz_point[2]/1000, 2)
        print (x,y,z)

        # for debug
        frame = rgb_image
        cv2.imshow('frame',frame)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            sys.exit()
 
def main():
    rospy.init_node('depth_teleop_turtle')
    DepthTeleopTurtle()
    rospy.spin()
 
if __name__ == "__main__":
    main()