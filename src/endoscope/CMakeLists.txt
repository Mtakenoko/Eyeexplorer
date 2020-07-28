cmake_minimum_required(VERSION 3.5)
project(endoscope)

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
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rcutils REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(PCL COMPONENTS common REQUIRED)
find_package(Ceres REQUIRED)

# cap_endoscope node
add_executable(cap_endoscope
src/cap_endoscope.cpp
option/options_cap_endoscope.cpp
)
ament_target_dependencies(cap_endoscope
"rclcpp"
"sensor_msgs"
"std_msgs"
"OpenCV"
)
install(TARGETS
cap_endoscope
DESTINATION lib/${PROJECT_NAME}
)

# reconstructor node
add_executable(reconstructor
  src/reconstructor.cpp
  src/Reconstruction.cpp
  src/triangulate.cpp
  src/cost_function.cpp
  option/options_reconstructor.cpp
  ../HTL/src/transform.cpp
  ../HTL/src/msg_converter.cpp
)
target_link_libraries(reconstructor
  ceres
)
ament_target_dependencies(reconstructor
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "std_msgs"
  "OpenCV"
  "message_filters"
  "ceres"
)
install(TARGETS reconstructor
  DESTINATION lib/${PROJECT_NAME}
)


# tracking_test node
add_executable(tracking
  src/tracking.cpp
)
ament_target_dependencies(tracking
  "rclcpp"
  "sensor_msgs"
  "OpenCV"
)
install(TARGETS tracking
  DESTINATION lib/${PROJECT_NAME}
)

# tracking_subscriber node
add_executable(tracking_subscriber
  src/tracking_subscriber2.cpp
)
ament_target_dependencies(tracking_subscriber
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "message_filters"
  "boost"
  "OpenCV"
)
install(TARGETS tracking_subscriber
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
