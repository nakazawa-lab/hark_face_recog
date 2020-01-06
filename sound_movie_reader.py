#!/usr/bin/env python
## coding: UTF-8

import os
import sys
import subprocess

os.chdir("/home/nvidia/catkin_ws/src/hark_face_recog/src/userdata/networks")
dataset_path = '/home/nvidia/catkin_ws/src/hark_face_recog/src/userdata/records/movie_dataset/face_localization_with_shokudo_noise/shokudo_noise_trimmed'
sound_data_path = os.path.join(dataset_path, "record8ch.wav")
movie_path = "/home/nvidia/catkin_ws/src/hark_face_recog/src/input/mp4file/"


e0 = "pkill -KILL -f 'roslaunch hark_face_recog dataset.launch'"
print(e0)

# コマンド1 & コマンド2 → バックグラウンドでコマンド1を実行しつつ、コマンド2を実行する
# コマンド1 && コマンド2 → コマンド1が正常終了したときのみコマンド2が実行される
e1 = 'roslaunch hark_face_recog dataset.launch &' + \
    'sleep 2 && ' + \
     './hark_ros_julius_main_modal.n ../config/tamago_geotf.zip ../config/tamago_geotf.zip ' +\
     '../records/sep_files/face_localization_with_shokudo_noise/record_ ../records/movie_dataset/sunagawa/go_multi.wav'
print(e1)

os.system(e0)
os.system(e1)


# command0 = ["pkill", "-KILL", "-f", "roslaunch hark_face_recog dataset.launch"]
# command1 = ["roslaunch hark_face_recog dataset.launch &"]
# command2 = ["sleep 2 &&"]
# command3 = ["./hark_ros_julius_main_modal.n", "../config/tamago_geotf.zip", "../config/tamago_geotf.zip", "../records/sep_files/face_localization_with_shokudo_noise/record_", "../records/movie_dataset/face_localization_with_shokudo_noise/shokudo_rec3/record8ch.wav "]
#
# subprocess.call(command0)
# subprocess.call(command1)
# subprocess.call(command2)
# subprocess.call(command3)


# print('')
        # file.write('\n')
