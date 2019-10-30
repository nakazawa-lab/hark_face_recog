# HARK(xavier)のセットアップ及び実行手順

　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　文責　長野　2019/10/2更新



## 概要

**HARK**(ロボット聴覚オープンソースソフトウェア)を使用し、音源定位、音源分離を行い、分離された音声を**Julius**(大語彙連続音声認識エンジン)を用いて音声認識する。また、**ROS**のトピック通信を利用してKinectv2で取得した話者の顔方向のデータをHARKに送ることによって、顔方向の音声のみを分離する。



## インストール方法

1. **HARK**のインストール([HARK公式サイト](https://www.hark.jp/install/linux/)を参照)

   **HARK**関連のモジュールをインストールするディレクトリを`/home/HARK`とする。

   **xavier**において、`hark-base`, `harkmw`, `libharkio3`, `harktool5`に関しては上記の公式サイトを参考にインストールできる。それ以外の`libhark-netapi`、`hark-core`、`hark-designer`に関しては公式サイトの手順通りにインストールをしようとするとエラーが発生するので、以下に具体的な解決法を示す

   

   ・`libhark-netapi`のmakeを実行する際に、`sys/io.h`ファイルが存在しないというエラーが出るので、`/home/nvidia/HARK/libhark-netapi-3.0.7/hark-netapi.c`内の`#include <sys/io.h>`をコメントアウトすることによって無事にmakeできるようになる。

   

   ・`hark-core`をmakeする際に、公式サイトにも書いてあるように、`openBLAS`のバグによりエラーが発生するので、cmakeを行う前に

   ```
   $ sudo apt purge libopenblas-dev
   ```

   を実行し、一旦`libopenblas-dev`を消した後、

   ```
   $ sudo apt install libopenblas-dev
   ```

   を実行することによって`libopenblas-dev`をインストールし直すとmakeができる。

   (注意点)原因は不明だがpurgeとinstallを数回繰り返さないとmakeできない場合がある

   

   ・`hark-designer`に関しては公式サイトの手順通りにインストールを行うと、arm64をサポートしていないというエラーが出てインストールできない。そこで、[このページ](http://archive.hark.jp/harkrepos/dists/bionic/non-free/source/)にあるソースファイル(`hark-designer_3.0.7.tar.xz`)をダウンロードし、展開した後、

   ```
   $ cd ~/hark-designer
   $ sudo apt-get update
   $ sudo apt-get install nodejs graphviz
   $ sudo npm install
   ```

   を実行する。`sudo npm install`を実行する際に、エラーが発生するが問題はない。その後、

   ```
   $ node app.js
   ```

   を実行し、`chromium`などのブラウザを立ち上げ、[http://localhost:3000](http://localhost:3000)にアクセスすることで`hark-designer`が起動する。

   `hark-designer`起動後、ネットワークファイルを実行すると、動作が重くなり画面がフリーズしてしまうので、

   ```
   $ gedit ~/.bashrc
   ```

   を実行し、`bashrc`を立ち上げて`export OPENBLAS_NUM_THREADS=1`を追記する。

   その後、端末を立ち上げ直し、`hark-designer`を起動すると問題なくネットワークファイルを実行することができる。

   以上が確認できれば、**HARK**のインストールは無事終了である。

   

2. **HARK-python**のインストール([このページ](https://www.hark.jp/document/packages/hark-python3/harkpython3.html)を参照)

   **デスクトップPC**には以下のコマンドだけで入る。

   ```
   $ sudo apt install hark-python3 python3-numpy python3-matplotlib
   ```

   **xavier**の場合、hark-python3はarchitectureの問題で`apt install`できないので、ソースをダウンロードし、コンパイルする必要がある。

   ソースは[このページ](http://archive.hark.jp/harkrepos/dists/bionic/non-free/source/)にあるので、ここから`hark-python3 3.0.7.tar.xz`をインストールし、ファイルをHARKのディレクトリ(`~/HARK`など)に解凍する。(なぜか解凍後のファイル名は`hark-python2`になるが気にしなくて良い)

   次に[pybind11](https://github.com/pybind/pybind11)を`~/HARK/hark-python2`直下にダウンロードし、

   ```
   $ cd ~/HARK/hark-python2/pybind11-master
   $ mkdir build
   $ cd build
   $ cmake -DPYBIND11_TEST=OFF ..
   $ make
   $ sudo make install
   ```

   を実行し、`pybind11`をインストールする。

   その後、

   ```
   $ cd ~/HARK/hark-python2
   $ mkdir build
   $ cd build
   $ cmake ..
   $ make
   $ sudo make install
   ```

   を実行して、`harkpython`をコンパイルする

   最後に

   ```
   $ cd harkpython
   $ sudo python3 setup.py install
   ```

   を実行して必要な`hark-python`モジュールをインストールする

   

3. **HARK-ros**をインストール

   まず最初に、

   ```
   $ cd ~/HARK
   $ apt source hark-ros-msgs-melodic
   ```

   を実行後、ROSのCMakeLists.txtを`~/HARK/hark-ros-msgs-melodic-3.0.7/src/`内にコピーし、

   ```
   $ cd hark-ros-msgs-melodic-3.0.7
   ```

   でディレクトリに入り、

   ```
   $ catkin_make
   ```

   を実行してソースファイルをコンパイルする。

   その後、

   ```
   $ source ~/HARK/hark-ros-msgs-melodic-3.0.7/devel/setup.sh
   ```

   を実行する。

   

   次に、

   ```
   $ cd ~/HARK
   $ apt source hark-ros-melodic
   ```

   を実行してソースをダウンロードし、

   ```
   $ mkdir build
   $ cd build
   $ source ~/HARK/hark-ros-msgs-melodic-3.0.7/devel/setup.sh
   $ cmake ..
   $ make
   $ sudo make install
   ```

   を実行する。

   

   hark-rosがインストールできたことを確認するには

   ```
   $　cd ~/HARK/hark-designer
   $　node app.js
   ```

   を実行した後、ブラウザを立ち上げ、[http://localhost:3000](http://localhost:3000)にアクセスすることによって`hark-designer`を立ち上げ、画面内の`Preferences`をクリックし、Packagesの部分にhark-ros.jsonが存在することを確認すれば良い。

   

   また、注意点としてhark-ros実行後、`Gtk-Message: Failed to load module "canberra-gtk-module"`というエラーが起きるので、

   ```
   $ sudo apt install libcanberra-gtk-module --reinstall
   ```

   を実行する必要がある。

   

4. **HARK用に作られたJulius**をインストール

   **デスクトップPC**には以下のコマンドだけで入る。

   ```
   $ sudo apt-get install julius-4.2.3-hark-plugin_3.0.7sudo apt-get install julius-4.2.3-hark-plugin
   ```

   **xavier**には上記コマンドでインストールできないので、[ソース](http://archive.hark.jp/harkrepos/dists/bionic/non-free/source/) から`julius-4.2.3-hark`と`julius-4.2.3-hark-plugin`をダウンロードし、コンパイルして使用する。

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

   →うまくいかない場合は、`config.guess`と`config.sub`が古い可能性があるので、[このサイト](https://qiita.com/gratori/items/c744f53088509dc07687)を参考にして、`config.guess`と`config.sub`を最新のものに置き換えれば良い

   

   次にjulius_pluginをインストールする

   ソースからダウンロードした`julius-4.2.3-hark-plugin`ディレクトリ内で以下のコマンドを実行すると`/usr/local/lib/julius_plugin`が作成される

   ```
   $ export JULIUS_SOURCE_DIR=../julius-4.2.3-hark
   $ make
   $ sudo make install
   ```

   しかし、`julius_plugin`は`/usr/lib/julius_plugin`になければjuliusが動作しないので、

   ```
   $ sudo cp -r /usr/local/lib/julius_plugin /usr/lib
   ```

   を実行することで、juliusが正常に動作するようになる

   

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

5. HARKのネットワークファイルを実行。第1引数と第2引数はそれぞれ音源定位、音源分離の伝達関数ファイル。第3引数は分離音声の保存ファイル名。

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

1. シェルスクリプトファイルで上記の実行コマンドを一括実行できるようにする。



## 参考

1. [音声認識ネットワークサンプル](https://www.hark.jp/download/samples/)
2. [音声認識ネットワークサンプルの使い方](https://www.hark.jp/document/3.0.0/hark-cookbook-ja/sect0020.html)
3. [音源定位、音源分離の伝達関数ファイル](https://www.hark.jp/document/supported/)
4. [HARKソースコード(コンパイルしてから使用)](http://archive.hark.jp/harkrepos/dists/bionic/non-free/source/)

