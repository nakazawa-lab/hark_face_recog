# -*- coding: utf-8 -*-
import wave
import numpy as np
import matplotlib.pyplot as plt
import sys

args = sys.argv

wf = wave.open(args[1] , "r" )
buf = wf.readframes(wf.getnframes())

# バイナリデータを整数型（16bit）に変換
data = np.frombuffer(buf, dtype="int16")

# グラフ化
plt.plot(data)
plt.grid()
plt.show()
