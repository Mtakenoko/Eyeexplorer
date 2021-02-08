cmake_minimum_required(VERSION 3.5)
project(experiment)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
  add_compile_options(-Wall -Wextra -Wpedantic -lpthread)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  add_compile_options(-Wall -Wextra -Wpedantic -lpthread)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -lpthread)
endif()

set(CMAKE_CXX_COMPILE_FLAGS "-std=c++11")
add_compile_options(-Wall -Wextra -Wpedantic -lpthread)

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

# include directory
include_directories(${OCTOMAP_INCLUDE_DIRS})

# OpenMP
if (OPENMP_FOUND) 
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

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

# calc_rmse node
add_executable(calc_rmse
  src/calcRMSE_main.cpp
  src/calcRMSE.cpp
)
ament_target_dependencies(calc_rmse
  "rclcpp"
  "sensor_msgs"
  "visualization_msgs"
  "OpenCV"
)
install(TARGETS
  calc_rmse
  DESTINATION lib/${PROJECT_NAME}
)

# pull_endoscope node
add_executable(pull_endoscope
  src/pullEndoscope_main.cpp
  src/pullEndoscope.cpp
)
ament_target_dependencies(pull_endoscope
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "visualization_msgs"
  "OpenCV"
)
install(TARGETS
  pull_endoscope
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
