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
find_package(message_filters)
find_package(OpenCV REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rcutils REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(PCL COMPONENTS common REQUIRED)

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
# cap_endoscope_large node
add_executable(cap_endoscope_large
src/cap_endoscope_large.cpp
option/options_cap_endoscope_large.cpp
)
ament_target_dependencies(cap_endoscope_large
"rclcpp"
"sensor_msgs"
"std_msgs"
"OpenCV"
)
install(TARGETS
cap_endoscope_large
DESTINATION lib/${PROJECT_NAME}
)

# matching node
add_executable(matching
  src/feature_matching.cpp
  option/options_feature_matching.cpp
)
ament_target_dependencies(matching
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
)
install(TARGETS
  matching
  DESTINATION lib/${PROJECT_NAME}
)

# orb matching node
add_executable(orb_matching
  src/orb_matching.cpp
  option/options_orb_matching.cpp
)
ament_target_dependencies(orb_matching
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
  "message_filters"
)
install(TARGETS
  orb_matching
  DESTINATION lib/${PROJECT_NAME}
)

# opticalflow node
add_executable(opticalflow
  src/opticalflow.cpp
)
ament_target_dependencies(opticalflow
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
)
install(TARGETS
  opticalflow
  DESTINATION lib/${PROJECT_NAME}
)

# matching & reconstruction node
add_executable(reconstruction
  src/matching_reconstruction.cpp
  option/options_reconstruction.cpp
)
# 実行ファイルをリンクする
target_link_libraries(reconstruction
  ktl 
  ktlrt
  ${Boost_SYSTEM_LIBRARY}
)

ament_target_dependencies(reconstruction
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
  "message_filters"
  "tf2_msgs"
  "tf2_geometry_msgs"
  "rcl_interfaces"
  "rcutils"
  "pcl_conversions"

)
install(TARGETS reconstruction
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
