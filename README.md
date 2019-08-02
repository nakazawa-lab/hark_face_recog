## 顔認識パッケージ
このファイルは顔認識をして認識した顔の方向を出力するパッケージです。

- face_recog_to_ROS.py：顔認識をdlibで行い、相対的なカメラ座標系(x, y, z)における顔の位置をROS通信システム上に送信します。
- dlib_module.py：dlibの顔認識についてまとめたファイルです。
- getXYZ.py：pythonで三次元位置情報を扱えるかどうか確かめるためのテスト用ファイルです。

## 依存ライブラリ
### HARKのインストール
[こちら](https://www.hark.jp/hark-ros-msgs-installation-instructions/)を参照してHARK関連のライブラリをインストールしてください。

***[注意]*** 一番最初のhark-ros-msgsに関しては下記のようにしてインストールするディレクトリを合わせたいと思います。

まずはhark-ros-msgs-melodicのインストール。
```
$ cd ~/HARKに関するディレクトリ
$ apt source hark-ros-msgs-melodic
```

その後、インストール場所を`catkin_ws/src`に指定。

```
$ cp -r ~/HARKに関するディレクトリ/hark-ros-msgs-melodic*/src/* ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

この`catkin_make`によってROS-HARK関連のヘッダーファイルは`catkin_ws/devel/include/hark_msgs`へとインストールされます。

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
$ rosrun hark_face_recog hark_face_recog_face_xyz_to_HARK_cpp
```

```
$ roscd hark_face_recog
$ cd src
$ batchflow hark_ros_main.n
```

### rosrunで実行したい場合
Pythonで書かれているのでROS上でPythonの実行ファイルとして認識させるには以下のコマンドを一度行う必要があります。

```
$ roscd hark_face_recog
$ cd src
$ chmod 755 face_recog_to_ROS.py
```

## PointCloudLibraryをROSで簡単に試す場合

テスト用にkinect画面上の中心点のxyzの値を取得するコードのサンプルもあります。
iai-kinect2を立ち上げた状態で以下のコマンドで実行できます。
```
$ roscd hark_face_recog
$ cd src
$ python getXYZ.py
```
