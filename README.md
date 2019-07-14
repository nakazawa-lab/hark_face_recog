## 顔認識パッケージ
このファイルは顔認識をして認識した顔の方向を出力するパッケージです。

## 使用方法
Pythonで書かれているのでROSのPythonを実行ファイルとして認識させる以下のコマンドを一度行う必要があります。

```
$ roscd hark_face_recog
$ cd src
$ chmod 755 getXYZ.py
```

実行コマンドは、

```
$ rosrun hark_face_recog getXYZ.py
```

です。
