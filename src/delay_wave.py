#!/usr/bin/env python
## coding: UTF-8

import rospy
import numpy as np

from hark_msgs.msg import HarkWave

class DelayWave:
    def __init__(self):
        self.harkwave_list = []
        self.DELAY_LENGTH = 20
        self._hark_wave_sub = rospy.Subscriber('/HarkWave', HarkWave, self.callback)
        self._delay_data_pub = rospy.Publisher('/HarkWaveDelay', HarkWave, queue_size=100000)

    def callback(self, hw):
        print('now frame: ' + str(hw.count))
        self.harkwave_list.append(hw)
        if len(self.harkwave_list)!=0:
            if hw.count - self.harkwave_list[0].count > self.DELAY_LENGTH:
                print(self.harkwave_list[0].count)
                self._delay_data_pub.publish(self.harkwave_list[0])
                self.harkwave_list.pop(0)

if __name__ == '__main__':
    rospy.init_node('delay_wave',anonymous=True)
    dw = DelayWave()
    rospy.loginfo('running..')
    rospy.spin()
