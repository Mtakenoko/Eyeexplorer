cmake_minimum_required(VERSION 3.5)
project(map)

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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(VTK 6.3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(PCL COMPONENTS common REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcutils REQUIRED)
find_package(Ceres REQUIRED)

# pointcloud_storage node
# スタンドアローンノードの実行ファイルをコンパイルする
add_executable(pointcloud_storage
  src/pointcloud_storage.cpp
)
# コンパイルターゲットに依存パッケージの情報を追加する
ament_target_dependencies(pointcloud_storage
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
  "tf2_msgs"
  "tf2_geometry_msgs"
  "rcl_interfaces"
  "rcutils"
  "pcl_conversions"
)
# 実行ファイルをインストール
install(TARGETS pointcloud_storage
  DESTINATION lib/${PROJECT_NAME}
)

# eyeball_estimator node
# スタンドアローンノードの実行ファイルをコンパイルする
add_executable(eyeball_estimator
  src/eyeball_estimator.cpp
)
# 実行ファイルをリンクする
target_link_libraries(eyeball_estimator
  ceres
)
# コンパイルターゲットに依存パッケージの情報を追加する
ament_target_dependencies(eyeball_estimator
  "rclcpp"
  "sensor_msgs"
  "visualization_msgs"
  "OpenCV"
  "ceres"
)
# 実行ファイルをインストール
install(TARGETS
  eyeball_estimator
  DESTINATION lib/${PROJECT_NAME}
)

# pointcloud_filter node
# スタンドアローンノードの実行ファイルをコンパイルする
add_executable(pointcloud_filter
  src/filter.cpp
  src/pointcloud_filter.cpp
)
target_link_libraries(pointcloud_filter
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
  ${PCL_LIBRARIES}
)
# コンパイルターゲットに依存パッケージの情報を追加する
ament_target_dependencies(pointcloud_filter
  "rclcpp"
  "sensor_msgs"
  "pcl_conversions"
  "VTK 6.3"
  "Eigen3"
  "PCL 1.8"
  "PCL COMPONENTS common"
)
# 実行ファイルをインストール
install(TARGETS
  pointcloud_filter
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
