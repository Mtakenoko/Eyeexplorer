cmake_minimum_required(VERSION 3.5)
project(eyemodel)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Boost REQUIRED system)
find_package(Ceres REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenMP)
find_package(cv_bridge)

#Qt5 configulation
set(CMAKE_AUTOMOC ON)
find_package(ament_cmake REQUIRED)
# find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
find_package(Qt5 REQUIRED
  Core
  Widgets
)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

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

# eyeball_estimator node
# スタンドアローンノードの実行ファイルをコンパイルする
add_executable(eyemodel_estimator
  src/eyeball_estimator_main.cpp
  src/eyeball_estimator.cpp
  src/MainDialog.cpp
)
# 実行ファイルをリンクする
target_link_libraries(eyemodel_estimator
  ceres
  ${QT_LIBRARIES}
)
# コンパイルターゲットに依存パッケージの情報を追加する
ament_target_dependencies(eyemodel_estimator
  "rclcpp"
  "sensor_msgs"
  "visualization_msgs"
  "OpenCV"
  "ceres"
)
# 実行ファイルをインストール
install(TARGETS
  eyemodel_estimator
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
