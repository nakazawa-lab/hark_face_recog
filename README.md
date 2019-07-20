## 顔認識パッケージ
このファイルは顔認識をして認識した顔の方向を出力するパッケージです。

- face_recog_to_ROS.py：顔認識をdlibで行い、カメラ座標系における顔の位置をROS通信システム上に送信します。
- face_xyz_to_HARK.py：顔の三次元位置情報をHARKへと送信します。
- dlib_module.py：dlibの顔認識についてまとめたファイルです。
- getXYZ.py：pythonで三次元位置情報を扱えるかどうか確かめるためのテスト用ファイルです。

## 使用方法
ターミナルを4つ開いて以下をそれぞれ実行。

```
$ roscore
```

```
$ roslaunch kinect2_bridge kinect2_bridge.launch
```

```
$ roscd hark_face_recog
$ cd src
$ python face_recog_to_ROS.py
```

```
$ roscd hark_face_recog
$ cd src
$ python face_xyz_to_HARK.py
```


### rosrunで実行したい場合
Pythonで書かれているのでROS上でPythonの実行ファイルとして認識させるには以下のコマンドを一度行う必要があります。

```
$ roscd hark_face_recog
$ cd src
$ chmod 755 face_xyz_to_HARK.py face_recog_to_ROS.py
```

## PointCloudLibraryをROSで簡単に試す場合

テスト用にkinect画面上の中心点のxyzの値を取得するコードのサンプルもあります。
iai-kinect2を立ち上げた状態で以下のコマンドで実行できます。
```
$ roscd hark_face_recog
$ cd src
$ python getXYZ.py
```
