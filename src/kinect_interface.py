#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class KinectInterface:
    def __init__(self):
        self._image_sub = rospy.Subscriber('/kinect2/hd/image_color', Image, self.callback)
        self._downsampling_pub = rospy.Publisher('dsampled_image', Image, queue_size=1)
        self.rate = rospy.Rate(20)

    def callback(self, data):
        self._downsampling_pub.publish(data)
        # self.display(data)
        self.rate.sleep()

    def display(self, img):
        cv_img = CvBridge().imgmsg_to_cv2(img, 'bgr8')
        cv2.imshow('img', cv_img)
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('kinect_interface',anonymous=True)
    KI = KinectInterface()
    rospy.spin()