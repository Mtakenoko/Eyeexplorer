cmake_minimum_required(VERSION 3.5)
project(arm_status)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
  add_compile_options(-Wall -Wextra -Wpedantic -lktl -lts01 -lktlrt -lpthread)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  add_compile_options(-Wall -Wextra -Wpedantic -lktl -lts01 -lktlrt -lpthread)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -lktl -lts01 -lktlrt -lpthread)
endif()

set(CMAKE_CXX_COMPILE_FLAGS "-std=c++11 -lktl")
add_compile_options(-Wall -Wextra -Wpedantic -lktl -lts01 -lktlrt -lpthread)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

# forward_kinematics node
# スタンドアローンノードの実行ファイルをコンパイルする
add_executable(forward_kinematics
  src/forward_kinematics.cpp
  src/encoder.cpp
  src/arm_ver3.cpp
)
# 実行ファイルをリンクする
target_link_libraries(forward_kinematics ts01 ktl ktlrt)
# コンパイルターゲットに依存パッケージの情報を追加する
ament_target_dependencies(forward_kinematics
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "OpenCV"
  "tf2_msgs"
  "tf2_geometry_msgs"
)
# 実行ファイルをインストール
install(TARGETS
  forward_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

# joint_publisher node
# スタンドアローンノードの実行ファイルをコンパイルする
add_executable(joint_publisher
  src/joint_publisher.cpp
  src/encoder.cpp
)
# 実行ファイルをリンクする
target_link_libraries(joint_publisher ts01 ktl ktlrt)
# コンパイルターゲットに依存パッケージの情報を追加する
ament_target_dependencies(joint_publisher
  "rclcpp"
  "sensor_msgs"
)
# 実行ファイルをインストール
install(TARGETS
  joint_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# arm_state_holder node
# スタンドアローンノードの実行ファイルをコンパイルする
add_executable(arm_state_holder
  src/arm_state_holder.cpp
)
# コンパイルターゲットに依存パッケージの情報を追加する
ament_target_dependencies(arm_state_holder
  "rclcpp"
  "geometry_msgs"
  "tf2_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
)
# 実行ファイルをインストール
install(TARGETS
  arm_state_holder
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
