# -*- coding: utf-8 -*-
import pyaudio  #録音機能を使うためのライブラリ
import wave     #wavファイルを扱うためのライブラリ

#オーディオデバイスの情報を取得、マイクのインデックス番号を入手する。
iAudio = pyaudio.PyAudio()
for x in range(0, iAudio.get_device_count()): 
    print("index: " + str(iAudio.get_device_info_by_index(x)["index"])
    + ", name: " + str(iAudio.get_device_info_by_index(x)["name"]))
RECORD_SECONDS = 5 #録音する時間の長さ（秒）
WAVE_OUTPUT_FILENAME = "sample.wav" #音声を保存するファイル名
iDeviceIndex = 4 #録音デバイスのインデックス番号

 
#基本情報の設定
FORMAT = pyaudio.paInt16 #音声のフォーマット
CHANNELS = 8             #モノラル
RATE = 16000             #サンプルレート
CHUNK = 1024            #データ点数
audio = pyaudio.PyAudio() #pyaudio.PyAudio()
 
stream = audio.open(format=FORMAT, channels=CHANNELS,
        rate=RATE, input=True,
        input_device_index = iDeviceIndex, #録音デバイスのインデックス番号
        frames_per_buffer=CHUNK)
 
#--------------録音開始---------------
 
print ("recording...")
frames = []
for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    frames.append(data)
 
 
print ("finished recording")
 
#--------------録音終了---------------
 
stream.stop_stream()
stream.close()
audio.terminate()
 
waveFile = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
waveFile.setnchannels(CHANNELS)
waveFile.setsampwidth(audio.get_sample_size(FORMAT))
waveFile.setframerate(RATE)
waveFile.writeframes(b''.join(frames))
waveFile.close()
