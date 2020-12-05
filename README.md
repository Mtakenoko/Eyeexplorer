# ros2_eyeexplorer
眼科用内視鏡を用いた三次元復元のためのレポジトリ

## Contents
- 眼科用内視鏡を用いた三次元復元
- EyeExplorerの運動学
- 眼科用内視鏡の自動切り抜き
- ARマーカーを用いたロボットの関節角等のパラメータ補正
- TS-01管理
- 仮想深度画像生成
  
などがあります

# Application & Package
## TS-01との接続
```
ros2 run ts01 ts01_manager
```
また下記ロボットアームの運動学などもまとめたのがこれ。これやったら下のlaunchを立ち上げる必要はない。
```
ros2 launch workspace/ros2_eyeexplorer/src/arm/arm_rviz/launch/eyeexplorer.launch.py
```

## Robot Arm
EyeExplorer3.0号機のURDFを用いた順運動学ノードの立ち上げ。各関節や内視鏡先端の位置姿勢がpublishされる。
```
ros2 launch workspace/ros2_eyeexplorer/src/arm/arm_rviz/launch/arm_FK.launch.py 
```
下はEyeExplorer3.0号機に内視鏡先端の曲げモデルも含めたURDFを用いた順運動学ノード
```
ros2 launch workspace/ros2_eyeexplorer/src/arm/arm_rviz/launch/arm_dif_FK.launch.py 
```

## 内視鏡キャプチャ
WEBカメラデバイスとして認識できる内視鏡映像キャプチャデバイスを使用。内視鏡特有の黒い部分を自動的に切り抜くように設定。320*320の画像を`/endoscope_image`としてpublish。
```
ros2 run endoscope cap_endoscope -c 2
```
またサイズ変更できるのもある（非推奨）
```
ros2 run endoscope cap_endoscope_large -c 2  -s 0 -rx 640 -ry 640 
```
### オプション
基本的には`-h`オプションをして確認してください。
``` 
 -h: This message.
 -r: Reliability QoS setting:
    0 - best effort
    1 - reliable (default)
 -d: Depth of the queue: only honored if used together with 'keep last'. 10 (default)
 -f: Publish frequency in Hz. 30 (default)
 -k: History QoS setting:
    0 - only store up to N samples, configurable via the queue depth (default)
    1 - keep all the samples
 -s: Camera stream:
    0 - Do not show the camera stream
    1 - Show the ROI camera stream
    2 - Show the Preprocessed camera stream
 -x WIDTH and -y HEIGHT. Resolution. 
    Please type v4l2-ctl --list-formats-ext 
    to obtain a list of valid values.
 -t TOPIC: use topic TOPIC instead of the default
 -c: Camera device
    Please type number of camera device.
 -m: produce images of endoscope's movie rather than connecting to a camera
```

## アームキャリブレーション
位置が既知のARマーカーが貼り付けられたボードを内視鏡画像で捉えることで、アームのエンコーダオフセット値を補正する。使用時はTS-01、カメラ映像、アームのノードを立ち上げてからやってください(roslaunchにしときます)。
```
ros2 run calibration arm_param_calibrator
```
使い方としては、<br>
「ARマーカを撮影→setボタン→新しいARマーカーを撮影→setボタン→（複数繰り返し）→calibrateボタン」<br>
みたいな感じ。結果は`calibration/Output/offset.txt`に出力されます。

## 時間キャリブレーション
TS-01からの入力とカメラ画像の入力の遅延時間をおおまかに計算するキャリブレーションソフト。
```
ros2 run calibration_delay delay_calibrator
```
Startボタンを押して内視鏡を動かしながらARマーカーを撮影します。ある程度データが溜まったらcaliblateボタンを押すと遅延時間が出力されます。

## 三次元復元
```
ros2 run endoscope reconstructor
```
### オプション
基本的には`-h`オプションをして確認してください。
```
 -h: This message.
 -r: Reliability QoS setting:
    0 - best effort
    1 - reliable (default)
 -d: Depth of the queue: only honored if used together with 'keep last'. 10 (default)
 -k: History QoS setting:
    0 - only store up to N samples, configurable via the queue depth (default)
    1 - keep all the samples
 --show: Show Matching scene:
    0 - Do not show the camera stream (default)
    1 - Show the camera stream
 --ceres-stdout: Show Matching scene:
    0 - Do not show (default)
    1 - Show the ceres stdout
 --mode: Select Mode:
    0 - Normal (default)
    1 - Eye
 --est-move: Estimation Movement.  
    0 - OFF (default)
    1 - ON
 --knn-ratio: Set Threshhold of knn matching ratio  
   (default) : 0.7f
 --thresh-ransac: Set Threshhold of RANSAC used for   
   (default) : 5.0
 --core: Set Number of CPU core used for Bundler
   (default) : 8 cores
 --scene: Set Number of Scenes used for triangulation
   (default) : 4 scenes
 --match: Set Matching method
   KNN          : 0
   BruteForce   : 1  (default)
 --extractor: Set Extractor method
   ORB    : 0
   AKAZE  : 1  (default)
   BRISK  : 2 
   SIFT   : 3 
   SURF   : 4 
   BRIEF  : 5 
```

## 眼球モデルとの距離計測
点群から平面や球面など指定したモデルを推定し、そこまでの距離を計測する。そして近ければ内視鏡抜去する信号をpublishする。
```
ros2 run map pullout_endoscope
```

### option
基本的には`-h`オプションをして確認してください。
```
 -h: This message.
 -r: Reliability QoS setting:
    0 - best effort
    1 - reliable (default)
 -d: Depth of the queue: only honored if used together with 'keep last'. 10 (default)
 -k: History QoS setting:
    0 - only store up to N samples, configur
    this->search(0.0, 0.0, 0.0);able via the queue depth (default)
    1 - keep all the samples
 --thresh_ransac: Set Threshhold of RANSAC  
   (default) : 5.0
 --core: Set Used CPU core for Bundler
   (default) : 8
 --model: Set Model
   PLANE        : 0 
   PLANE_RANSAC : 1 (defalut)
   SPHERE       : 2 
 --distance: Set distance of safety zone to avoide enfoscope from eye-ball
   (default) : 0.005
```

## 占有確率マップの作成
内視鏡深度画像と順運動学での位置姿勢を用いてワールド座標系での占有確率マップを作成する
```
ros2 run map gridmap_creator
```
### option
基本的には`-h`オプションをして確認してください。
```
 -h: This message.
 -r: Reliability QoS setting:
    0 - best effort
    1 - reliable (default)
 -d: Depth of the queue: only honored if used together with 'keep last'. 10 (default)
 -k: History QoS setting:
    0 - only store up to N samples, configurable via the queue depth (default)
    1 - keep all the samples
 --clamp-max: sets the maximum threshold for occupancy clamping (sensor model)
   (default) : 0.971
 --clamp-min: sets the minimum threshold for occupancy clamping (sensor model)
   (default) : 0.1192
 --occ: sets the threshold for occupancy (sensor model)
   (default) : 0.5
 --prob-hit: sets the probability for a -hit- (will be converted to logodds) - sensor model
   (default) : 0.7
 --prob-miss: sets the probability for a -miss- (will be converted to logodds) - sensor model
   (default) : 0.4
 --resol: Change the resolution of the octree, scaling all voxels.
          This will not preserve the (metric) scale!
   (default) : 0.001
```


## depth_image
DenseDepthなどのDNNを用いて深度推定を行うために、学習用のデータセットを作るためのソフトウェア。Captureボタンを押しデプス画像がうまく生成されていればSaveボタンを押す。深度画像の出力は`/depth_image/Output`内の各日時毎に生成されたディレクトリに登録順に保存される。
```
ros2 run depth_image depth_image_creator 
```
ここでは位置やスケールなどのパラメータが既知のモデルを用いて、仮想的な深度画像を生成する。その際用いられるモデルは`visualization_msgs::msg::Marker`で定義されているものを用いること。現在は楕円体のみ対応しています。
### LSTM用のデータセット作成
ConvLSTMに対応するため、画像シーケンスのデータセット作成を行うソフトウェアはこちら。depth_imageと同じような操作で作成することができる。
デフォルトでは10枚で1シーケンスを構成しており、5フレーム間隔でキーフレームを取得している。
```
ros2 run depth_image_seq depth_image_creator 
```
### データセット用のrosbag2
データセット作成用のrosbag2を用意しています。下記は内視鏡の明るさ設定を2段階目にして行っています。
```
ros2 bag play workspace/ros2_eyeexplorer/rosbag2/depth_create0.bag/
```

## depth_predict
```
python /home/takeyama/workspace/ros2_eyeexplorer/src/depth_predict/densedepth/predict.py
```

## Rviz2
```
rviz2 workspace/ros2_eyeexplorer/rviz2/eyeexplorer.rviz 
rviz2 workspace/ros2_eyeexplorer/rviz2/eyeexplorer2.rviz 
```

## venv
DenseDpethを動かすための仮想環境の立ち上げ
```
source /home/takeyama/workspace/my_env/bin/activate
```
仮想環境終了コマンド
```
deactivate
```

## いろいろなrosbagデータ
読み込むときはこんな感じ
```
ros2 bag play workspace/ros2_eyeexplorer/rosbag2/0923_eye1.bag
ros2 bag play workspace/ros2_eyeexplorer/rosbag2/depth_create0.bag/
```
逆に記録するときはこんな感じ。基本的に取るべきトピックは`/joiont_states`と`/endoscope_image`の2つでOK。
```
ros2 bag record -o  workspace/ros2_eyeexplorer/rosbag2/hoge.bag /joint_states /endoscope_image
```
