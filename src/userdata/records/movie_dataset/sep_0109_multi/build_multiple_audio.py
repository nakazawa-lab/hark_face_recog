# coding: UTF-8

import os
# from pydub import AudioSegment
here_path = os.getcwd()
dirs = os.listdir(here_path)

commands = os.listdir(here_path + '/suzuki/')

for subject in dirs:
    for command in commands:
        dts_path = here_path + '/' + subject + '/' + command

        # exec0 = 'sox -M ' + dts_path + '/*record*.wav ' + \
        #     here_path + '/' + subject + '/' + subject + '_' + command + '_multi.wav'
        # print(exec0)
        # os.system(exec0)

        if not '.wav' in dts_path:
            exec1 = 'cp ' + dts_path + '/*.mp4 ' + here_path + '/' + \
                subject + '/' + subject + '_' + command + '.mp4'
            print(exec1)
            os.system(exec1)

        # if '.wav' in dts_path:
        #     exec2 = 'rm ' + dts_path
        #     print(exec2)
