# ros2_eyeexplorer
・眼科用内視鏡を用いた三次元復元<br>
・ARマーカーを用いたロボットの関節角等のパラメータ補正<br>
・TS-01とのinput/output<br>
などなど<br>

各パッケージやノード、オプションなどの詳細なコマンドはcommand.mdを参照のこと

# Application & Package
## TS-01との接続
```
ros2 run ts01 ts01_sensor
```
EyeExplorer用の設定です。
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
ros2 run endoscope cap_endoscope -c 2 -s 1
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
 --publish: Set Publish pointcloud
   NORMAL        : 0 
   NORMAL_HOLD   : 1 
   BUNDLE        : 2 
   BUNDLE_HOLD   : 3 
   FILTER        : 4 
   FILTER_HOLD   : 5 (default)
   ESTIMATE      : 6 
   ESTIMATE_HOLD : 7 
```
## map
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
    0 - only store up to N samples, configurable via the queue depth (default)
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

## Rviz2
```
rviz2 workspace/ros2_eyeexplorer/rviz2/eyeexplorer.rviz 
```


## いろいろなrosbagデータ
読み込むときはこんな感じ
```
ros2 bag play workspace/ros2_eyeexplorer/rosbag2/calib_flower.bag
```
逆に記録するときはこんな感じ。基本的に取るべきトピックは`/ts01_encoder`と`/endoscope_image`の2つでOK。
```
ros2 bag record -o  workspace/ros2_eyeexplorer/rosbag2/hoge.bag /ts01_encoder /endoscope_image
```

