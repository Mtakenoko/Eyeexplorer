# TS01管理

## Usage
下記ノードを立ち上げることでEyeExplorer用のTS01が立ち上がる。
```
ros2 run ts01 ts01_manager
```
もっと汎用的にしたいのであれば、各ポートにどのようなセンサがつながるのか、そのセンサのconfigをyamlファイルに記述し、それに合わせてセンサ値をトピックにpubするようにしたい。けどやる気はありません。

## Installation
### TS01ライブラリのインストール
```
$ sudo apt-get install autoconf
$ sudo apt-get install libtool
$ sudo apt-get install make 
$ git clone ssh://flucome.pi.titech.ac.jp/git/ts01
$ autoreconf
$ ./configure
$ make
$ sudo make install
```