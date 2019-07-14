## 顔認識パッケージ
このファイルは顔認識をして認識した顔の方向を出力するパッケージです。

## 使用方法
Pythonで書かれているのでROSのPythonを実行ファイルとして認識させる以下のコマンドを一度行う必要があります。

```
$ roscd nakbot_ROS
$ cd src/face_recog
$ chmod 755 getXYZ.py
```

実行コマンドは、

```
$ rosrun nakbot_ROS getXYZ.py
```

です。
