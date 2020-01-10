#!/usr/bin/env python
## coding: UTF-8

import os
import sys

os.chdir("/home/nvidia/catkin_ws/src/hark_face_recog/src/userdata/networks")
cmd_array = ['come', 'go', 'right', 'left', 'no', 'nakbot', 'sleep']
dataset_path = '/home/nvidia/projects/dataset2/dataset_trim/'
output_path = '/home/nvidia/Desktop/order.txt'
file = open(output_path, 'w')
movie_path = "/home/nvidia/catkin_ws/src/hark_face_recog/src/input/mp4file/"

subjects = os.listdir(dataset_path)
for subject in subjects:
    for cmd in cmd_array:
        file.write(subject + '-' + cmd)

        e0 = "pkill -KILL -f 'roslaunch hark_face_recog dataset.launch'"
        print(e0)

        e1 = 'rm ' + movie_path + '*'
        print(e1)

        e2 = 'cp ' + dataset_path + subject + '/' + subject + '_' + cmd + '.mp4 ' + movie_path
        print(e2)

        e3 = 'roslaunch hark_face_recog dataset.launch &' + \
            'sleep 2 && ' + \
             './main_modal.n ../config/tamago_geotf.zip ../config/tamago_geotf.zip ' \
             '../records/sep_files/' + subject + '_' + cmd + ' ' \
             + dataset_path + subject + '/' + cmd + "*.wav "
        print(e3)

        os.system(e0)
        os.system(e1)
        os.system(e2)
        os.system(e3)

        print('')
        file.write('\n')

e4 = 'roslaunch hark_face_recog dataset.launch'
os.system(e4)
os.system(e0)