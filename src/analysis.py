#!/usr/bin/env python
## coding: UTF-8

import os

here_path = os.path.dirname(__file__)
if here_path == "":
    here_path = "."
inputs_path = here_path + '/input/inputs_to_julius/'
path = inputs_path + 'analysis.txt'
file = open(path, 'w')

dts = os.listdir(inputs_path)
dts.sort()
print(dts)
for dt in dts:
    if '.wav' in dt:
        text = dt + '\n'
        file.write(text)
file.close()