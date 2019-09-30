# 音源定位、音源分離、音声認識機能実行手順

　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　文責　長野　9/27更新



## 概要

**HARK**(ロボット聴覚オープンソースソフトウェア)を使用し、音源定位、音源分離を行い、分離された音声を**Julius**(大語彙連続音声認識エンジン)を用いて音声認識する。また、**ROS**のトピック通信を利用してKinectv2で取得した話者の顔方向のデータをHARKに送ることによって、顔方向の音声のみを分離する。



## インストール方法

1. HARKのインストール([HARK公式サイト](https://www.hark.jp/install/linux/)を参照)

   xavierは`hark-base`, `harkmw`, `libharkio3`, `harktool5`は上記の公式サイトを参考にインストールできる。
   
   ・`libhark-netapi`

2. HARK用に作られたJuliusをインストール

   デスクトップPCには以下のコマンドだけで入る。

   ```
   $ sudo apt-get install julius-4.2.3-hark-plugin_3.0.7sudo apt-get install julius-4.2.3-hark-plugin
   ```

   xavierには上記コマンドでインストールできないので、[ソース](http://archive.hark.jp/harkrepos/dists/bionic/non-free/source/) からダウンロードし、コンパイルして使用する。

   ダウンロードした、`julius-4.2.3-hark`ディレクトリ内で以下のコマンドを実行する。

   ```
   $ ./configure --prefix=/usr --enable-mfcnet --build=aarch64-unknown-linux-gnu --host aarch64-unknown-linux-gnu
   $ make
   $ sudo make install
   ```

   --buildと--hostに`aarch64-unknown-linux-gnu`を指定することに注意

   以下のコマンドを実行して、下記のように表示されたらjuliusのインストールは正常に終了している

   ```
   $ /usr/bin/julius_mft
   Julius rev.4.2.3 - based on 
   JuliusLib rev.4.2.3 (fast)  built for aarch64-unknown-linux-gnu
   Copyright (c) 1991-2013 Kawahara Lab., Kyoto University
   Copyright (c) 1997-2000 Information-technology Promotion Agency, Japan
   Copyright (c) 2000-2005 Shikano Lab., Nara Institute of Science and Technology
   Copyright (c) 2005-2013 Julius project team, Nagoya Institute of Technology
   Try '-setting' for built-in engine configuration.
   Try '-help' for run time options.
   ```

   →うまくいかない場合は、`config.guess`と`config.sub`が古い場合があるので、[このサイト](https://qiita.com/gratori/items/c744f53088509dc07687)を参考にして、`config.guess`と`config.sub`を最新のものに置き換えれば良い

   

   次にjulius_pluginをインストールする

   ソースからダウンロードした`julius-4.2.3-hark-plugin`ディレクトリ内で以下のコマンドを実行すると`/usr/local/lib/julius_plugin`が作成される

   ```
   $ export JULIUS_SOURCE_DIR=../julius_4.2.3-hark
   $ make
   $ sudo make install
   ```

   しかし、`julius_plugin`は`/usr/lib/julius_plugin`になければjuliusが動作しないので、

   ```
   $ sudo cp -r /usr/local/lib/julius_plugin /usr/lib
   ```

   を実行することで、juliusが正常に動作するようになる

   

3. (他の必要パッケージ(HARK-ROS, HARK-python)などのインストール方法もまとめる予定)



## ディレクトリ構成

必要なファイルやディレクトリは以下の通り。

```
userdata
├── networks
│   ├── hark_ros_julius_main.n (メインファイル)
│   └── hark_ros_process.py (音源定位方向を処理するファイル)
├── config   
│   └── tamago_geotf.zip (音源定位、音源分離伝達関数ファイル)  
├── records
│	└── sep_files (音源分離した音声を格納するディレクトリ)
└── speech_recognition
    ├── AM (音響モデル)
    │　　├── allTriphones
    │　　└── hmmdefs.gz
    ├── LM (言語モデル)
    │　　└── 省略
    ├── julius.jconf (Julius設定ファイル)
    └── result.txt (音声認識結果出力ファイル)
```



## 実行コマンド

ターミナルを6つ開き以下を実行

1. rosを起動。

   ```
   $ roscore
   ```

2. kinect1を起動。

   ```
   $ roslaunch kinect2_bridge kinect2_bridge.launch
   ```

3. 顔認識プログラムを起動。

   ```
   $ roscd hark_face_recog
   $ cd src
   $ python face_recog_to_ROS.py
   ```

4. 顔方向データをHARKへ送信するプログラムを起動

   ```
   $ rosrun hark_face_recog hark_face_recog_face_xyz_to_HARK_cpp
   ```

5. HARKのネットワークファイルを実行。第1引数と第2引数はそれぞれ音源定位、音源分離の伝達関数ファイル。第3引数は分離音声の保存ファイル名。それぞれの引数は絶対パスでなければエラーが出る。

   ```
   $ cd userdata/networks
   $ ./hark_ros_julius_main.n ../config/tamago_geotf.zip ../config/tamago_geotf.zip ../records/sep_files/record
   ```

6. Julius_mftを起動しつつ、音声認識結果を`result.txt`に書き込む。

   ```
   $ cd userdata/speech_recognition
   $ julius_mft -C julius.jconf | tee result.txt
   ```



## 今後やるべきこと

3. サンプルのようにシェルスクリプトファイルで上記のコマンドを一括実行できるようにする。



## 参考

1. [音声認識ネットワークサンプル](https://www.hark.jp/download/samples/)
2. [音声認識ネットワークサンプルの使い方](https://www.hark.jp/document/3.0.0/hark-cookbook-ja/sect0020.html)
3. [音源定位、音源分離の伝達関数ファイル](https://www.hark.jp/document/supported/)
4. [HARKソースコード(コンパイルしてから使用)](http://archive.hark.jp/harkrepos/dists/bionic/non-free/source/)

