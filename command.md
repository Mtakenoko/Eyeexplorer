# コマンド覚書
古いやつも一杯あるので注意

## ts01 pkg
```
ros2 run ts01 ts01_sensor
```
## endoscope pkg
```
ros2 run endoscope cap_endoscope -c 2  -s 0<
ros2 run endoscope cap_endoscope_large -c 2  -s 0
ros2 run endoscope cap_endoscope_large -c 2  -s 0 -rx 640 -ry 640 
ros2 run endoscope reconstruction_online_tf2 -s 1 -e 0
ros2 run endoscope reconstruction -r 0 -s 1 -e 0
ros2 run endoscope opticalflow
ros2 run image_tools showimage -t /endoscope_image
```

## arm pkg
```
ros2 run arm arm_fk
ros2 run arm_status arm_forward_kinematics
```

## stereo pkg
```
ros2 run stereo cap_stereo  -cl 2 -cr 3 -s 1
ros2 run stereo stereo_reconstruction  -r 0 -s 1 -q 15 -e 0
ros2 run stereo stereo_vision -s 1
```

## rosbag
```
ros2 bag play topic_endoscope_arm4.bag
ros2 bag play rosbag2/topic_endoscope_arm8.bag
ros2 bag play rosbag2/topic_stereo_cam.bag 
ros2 bag play rosbag2/topic_bandori3.bag
ros2 bag play rosbag2/topic_endoscope_ba.bag
ros2 bag record -o topic_endoscope_arm9.bag /ts01_encoder /endoscope_image /arm_trans /joint_states
ros2 bag record -o topic_endoscope_ba.bag /endoscope_image /joint_states /tf /endoscope_transform
```

## roslaunch
```
ros2 launch workspace/ros2_eyeexplorer/src/ros2/demos/dummy_robot/dummy_robot_bringup/launch/dummy_robot_bringup.launch.py
ros2 launch workspace/ros2_eyeexplorer/src/arm/arm_rviz/launch/arm_rviz.launch.py
ros2 launch workspace/ros2_eyeexplorer/src/arm/arm_rviz/launch/arm_rviz_noFK.launch.py
```

## colcon build
```
colcon build --symlink-install --packages-select arm_rviz arm_status map_server endoscope ts01 stereo realtime_test map test qt
```

# 九大用
```
ros2 run ts01 ts01_sensor
ros2 run endoscope cap_endoscope -c 2 -s 1
ros2 launch workspace/ros2_eyeexplorer/src/arm/arm_rviz/launch/arm_rviz.launch.py
ros2 run endoscope reconstruction_online -s 1 -e 0 -p 1
ros2 run ts01 ts01_AI_listener 
ros2 bag record -o rosbag2/topic_kyusyu_univ1.bag /ts01_encoder /ts01_ai /ts01_di /tf /tf_static /endoscope_image /arm_trans /joint_states
ros2 bag record -o rosbag2/topic_kyusyu_univ1.bag /ts01_encoder /endoscope_image /arm_trans /joint_states
```

# 
```
ros2 run ts01 ts01_sensor
ros2 run endoscope cap_endoscope -c 2 -s 1
ros2 launch workspace/ros2_eyeexplorer/src/arm/arm_rviz/launch/arm_rviz_noFK.launch.py
ros2 run endoscope reconstruction_online_delay -s 1 -e 0 -p 1
ros2 run endoscope reconstruction_BA -s 1
rviz2 workspace/ros2_eyeexplorer/rviz2/eyeexplorer.rviz
ros2 launch workspace/ros2_eyeexplorer/src/test/launch/pcl_remove.launch.py
```

# その他
## Ceres Solverのサンプル
```
ros2 run test ceres_test2 src/test/data/problem-16-22106-pre.txt
```

## 2020-4-19
```
ros2 launch workspace/ros2_eyeexplorer/src/arm/arm_rviz/launch/arm_FK.launch.py 
ros2 bag play rosbag2/topic_BA_mario.bag/
ros2 run endoscope reconstruction_BA -s 1
```

## 2020-6-2
```
ros2 bag play rosbag2/topic_BA_mario.bag/
ros2 launch workspace/ros2_eyeexplorer/src/arm/arm_rviz/launch/arm_FK2.launch.py 
ros2 run endoscope reconstruction_BA_class
ros2 run calibration arm_param_calibrator
```

## 
```
ros2 run ts01 ts01_sensor
ros2 run endoscope cap_endoscope -c 2 -s 1
ros2 launch workspace/ros2_eyeexplorer/src/arm/arm_rviz/launch/arm_FK2.launch.py
ros2 run calibration arm_param_calibrator
```

## 2020-8-11
まずはts01_managerを正しく動作させること。とりあえずこれを動かしてblockingされてないか確認する。
```
ros2 run ts01 ts01_manager
```
ブロッキングされてないなら、今度は以下を実行しts01_managerにてsubscriberが正しく動作しているか確認。
```
ros2 run endoscope cap_endoscope -c 2 -s 1
ros2 launch workspace/ros2_eyeexplorer/src/arm/arm_rviz/launch/arm_dif_FK.launch.py
ros2 run endoscope reconstructor --scene 4 --est-move 0 --match 1 --show 1 --mode 1
ros2 run map pullout_endoscope
```
もし以上にて正しく動作することを確認できれば、下記について取り組みたい
- 移動量推定を実際に行ってみてどんな感じか確認
- ステージを管理するライブラリ作成
- テクスチャマッピングについて調査・実装
