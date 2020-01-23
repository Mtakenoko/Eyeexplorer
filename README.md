# ros2_eyeexplorer
//////////
//ts01 pkg
ros2 run ts01 ts01_sensor

//endoscope pkg
ros2 run endoscope cap_endoscope -c 2  -s 0
ros2 run endoscope cap_endoscope_large -c 2  -s 0 
ros2 run endoscope cap_endoscope_large -c 2  -s 0 -rx 640 -ry 640 
ros2 run endoscope reconstruction -r 0 -s 1 -e 0
ros2 run endoscope opticalflow
ros2 run image_tools showimage -t /endoscope_image

//arm pkg
ros2 run arm arm_fk
ros2 run arm_status arm_forward_kinematics

//stereo pkg
ros2 run stereo cap_stereo  -cl 2 -cr 3 -s 1
ros2 run stereo stereo_reconstruction  -r 0 -s 1 -q 15 -e 0
ros2 run stereo stereo_vision -s 1

//rosbag
ros2 bag play topic_endoscope_arm4.bag
ros2 bag play rosbag2/topic_endoscope_arm8.bag
ros2 bag play rosbag2/topic_stereo_cam.bag 
ros2 bag play rosbag2/topic_bandori3.bag
ros2 bag record -o topic_endoscope_arm9.bag /ts01_encoder /endoscope_image /arm_trans /joint_states

//roslaunch
ros2 launch workspace/ros2_eyeexplorer/src/ros2/demos/dummy_robot/dummy_robot_bringup/launch/dummy_robot_bringup.launch.py
ros2 launch workspace/ros2_eyeexplorer/src/arm/arm_rviz/launch/arm_rviz.launch.py
ros2 launch workspace/ros2_eyeexplorer/src/arm/arm_rviz/launch/arm_rviz_noFK.launch.py

//colcon build
colcon build --symlink-install --packages-select arm_rviz arm_status map_server endoscope ts01 stereo realtime_test map test qt

//九大用
ros2 run ts01 ts01_sensor
ros2 run endoscope cap_endoscope -c 2 -s 1
ros2 launch workspace/ros2_eyeexplorer/src/arm/arm_rviz/launch/arm_rviz.launch.py
ros2 run endoscope reconstruction_online -s 1 -e 0 -p 1
ros2 run ts01 ts01_AI_listener 
ros2 bag record -o rosbag2/topic_kyusyu_univ1.bag /ts01_encoder /ts01_ai /ts01_di /tf /tf_static /endoscope_image /arm_trans /joint_states
ros2 bag record -o rosbag2/topic_kyusyu_univ1.bag /ts01_encoder /endoscope_image /arm_trans /joint_states