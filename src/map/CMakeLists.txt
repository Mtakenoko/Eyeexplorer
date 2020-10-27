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
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(PCL REQUIRED)
# find_package(pcl_conversions REQUIRED)
find_package(Boost REQUIRED system)
find_package(Ceres REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenMP)
find_package(cv_bridge)

# include directory
include_directories(${OCTOMAP_INCLUDE_DIRS})

# OpenMP
if (OPENMP_FOUND) 
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# PCL
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

# # eyeball_estimator node
# # スタンドアローンノードの実行ファイルをコンパイルする
# add_executable(eyeball_estimator
#   src/eyeball_estimator.cpp
# )
# # 実行ファイルをリンクする
# target_link_libraries(eyeball_estimator
#   ceres
# )
# # コンパイルターゲットに依存パッケージの情報を追加する
# ament_target_dependencies(eyeball_estimator
#   "rclcpp"
#   "sensor_msgs"
#   "visualization_msgs"
#   "OpenCV"
#   "ceres"
# )
# # 実行ファイルをインストール
# install(TARGETS
#   eyeball_estimator
#   DESTINATION lib/${PROJECT_NAME}
# )

# eyeball_estimator_insertion_point node
# スタンドアローンノードの実行ファイルをコンパイルする
# add_executable(eyeball_estimator_insertion_point
#   src/eyeball_estimator.cpp
# )
# # コンパイルターゲットに依存パッケージの情報を追加する
# ament_target_dependencies(eyeball_estimator_insertion_point
#   "rclcpp"
#   "sensor_msgs"
#   "visualization_msgs"
#   "OpenCV"
# )
# # 実行ファイルをインストール
# install(TARGETS
#   eyeball_estimator_insertion_point
#   DESTINATION lib/${PROJECT_NAME}
# )

# # pointcloud_filter node
# # スタンドアローンノードの実行ファイルをコンパイルする
# add_executable(pointcloud_filter
#   src/filter.cpp
#   src/pointcloud_filter.cpp
# )
# target_link_libraries(pointcloud_filter
#   ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
#   ${PCL_LIBRARIES}
# )
# # コンパイルターゲットに依存パッケージの情報を追加する
# ament_target_dependencies(pointcloud_filter
#   "rclcpp"
#   "sensor_msgs"
#   "pcl_conversions"
# )
# # 実行ファイルをインストール
# install(TARGETS
#   pointcloud_filter
#   DESTINATION lib/${PROJECT_NAME}
# )

# # pullout_endoscope node
# # スタンドアローンノードの実行ファイルをコンパイルする
# add_executable(pullout_endoscope
#   src/puller.cpp
#   src/pullout_endoscope.cpp
#   option/options_pullout.cpp
# )
# target_link_libraries(pullout_endoscope
#   ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
#   ${PCL_LIBRARIES}
# )
# # コンパイルターゲットに依存パッケージの情報を追加する
# ament_target_dependencies(pullout_endoscope
#   "rclcpp"
#   "sensor_msgs"
#   "geometry_msgs"
#   "message_filters"
#   "OpenCV"
# )
# # 実行ファイルをインストール
# install(TARGETS
#   pullout_endoscope
#   DESTINATION lib/${PROJECT_NAME}
# )

# # pullout node
# # スタンドアローンノードの実行ファイルをコンパイルする
# add_executable(pullout
#   src/pullout.cpp
# )
# # コンパイルターゲットに依存パッケージの情報を追加する
# ament_target_dependencies(pullout
#   "rclcpp"
#   "std_msgs"
#   "geometry_msgs"
#   "visualization_msgs"
#   "OpenCV"
# )
# # 実行ファイルをインストール
# install(TARGETS
#   pullout
#   DESTINATION lib/${PROJECT_NAME}
# )

# eyeball_publisher node
# スタンドアローンノードの実行ファイルをコンパイルする
add_executable(eyeball_publisher
  src/eyeball_publisher.cpp
)
# コンパイルターゲットに依存パッケージの情報を追加する
ament_target_dependencies(eyeball_publisher
  "rclcpp"
  "rcl_interfaces"
  "rcutils"
  "std_msgs"
  "visualization_msgs"
)
target_link_libraries(eyeball_publisher
  ${Boost_SYSTEM_LIBRARY}
)
# 実行ファイルをインストール
install(TARGETS eyeball_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# gridmap_creator node
add_executable(gridmap_creator
  src/gridmap_creator.cpp
)
target_link_libraries(gridmap_creator 
  ${OCTOMAP_LIBRARIES}
)

ament_target_dependencies(gridmap_creator
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "octomap"
  "OpenCV"
  "OpenMP"
  "cv_bridge"
)
install(TARGETS
  gridmap_creator
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
