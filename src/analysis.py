#!/usr/bin/env python
## coding: UTF-8

import os
from datetime import datetime

here_path = os.path.dirname(__file__)
if here_path == "":
    here_path = "."
inputs_path = here_path + '/input/inputs_to_julius/'
path = inputs_path + 'analysis.txt'
xpath = inputs_path + 'analysis_to_excel.txt'
file = open(path, 'w')
xfile = open(xpath, 'w')

dts = os.listdir(inputs_path)
dts.sort()
print(dts)


text = 'subject|command|refer_to|filename|time' + '\n'
xfile.write(text)
commands = ['go', 'back', 'right', 'left', 'come']

for dt in dts:
    if '.wav' in dt:
        time = os.path.getmtime(inputs_path + dt)
        loc = datetime.fromtimestamp(time)
        # print(loc)

        text = inputs_path + dt + '\n'
        file.write(text)

        index = dt.find('_')
        for cmd in commands:
            if cmd in dt:
                cmd_text = cmd
                break
            else:
                cmd_text = '[EROOR] Could not find command'
        if '100' in dt:
            ref = "face"
        else:
            ref = "audio"
        text = dt[:index] + '|' + cmd_text + '|' + ref + '|' + dt + '|' + str(time) + '\n'
        xfile.write(text)
file.close()
xfile.close()