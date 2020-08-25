cmake_minimum_required(VERSION 3.5)
project(calibration_delay)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
  add_compile_options(-Wall -Wextra -Wpedantic -lktl -lts01 -lktlrt -lpthread -lboost_system)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  add_compile_options(-Wall -Wextra -Wpedantic -lktl -lts01 -lktlrt -lpthread -lboost_system)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -lktl -lts01 -lktlrt -lpthread -lboost_system)
endif()

set(CMAKE_CXX_COMPILE_FLAGS "-std=c++11 -lktl")
add_compile_options(-Wall -Wextra -Wpedantic -lktl -lts01 -lktlrt -lpthread -lboost_system)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rcutils REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(PCL COMPONENTS common REQUIRED)
find_package(Ceres REQUIRED)

#Qt5 configulation
set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
# find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
find_package(Qt5 REQUIRED
  Core
  Widgets
)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

################  node setting  ###############
# delay_calibrator node
add_executable(delay_calibrator
  src/delay_calibrator.cpp
  src/arm.cpp
  src/MainDialog.cpp
  src/calibrate.cpp
  src/cost_function.cpp
  src/Scene.cpp
)
ament_target_dependencies(delay_calibrator
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "std_msgs"
  "OpenCV"
  "OpenMP"
  "ceres"
)
target_link_libraries(delay_calibrator
  ceres ts01 ktl ktlrt
  ${QT_LIBRARIES}
)
install(TARGETS delay_calibrator
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
