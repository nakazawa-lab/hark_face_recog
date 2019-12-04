#!/bin/sh

cd  ~/catkin_ws/src/hark_face_recog/src/userdata/speech_recognition/ && bash 1_Julius.sh & \
sleep 1 && cd ~/projects/kei_JetsonXavier/JetSAS/bin/Debug && sudo ./JetSAS & \

