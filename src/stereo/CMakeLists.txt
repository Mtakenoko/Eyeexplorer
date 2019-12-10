cmake_minimum_required(VERSION 3.5)
project(stereo)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# cap_stereo node
add_executable(cap_stereo
src/cap_stereo.cpp
option/options_cap_stereo.cpp
)
ament_target_dependencies(cap_stereo
"rclcpp"
"sensor_msgs"
"std_msgs"
"OpenCV"
)
install(TARGETS
cap_stereo
DESTINATION lib/${PROJECT_NAME}
)

# stereo_stereo_reconstruction node
add_executable(stereo_reconstruction
  src/stereo_reconstruction.cpp
  option/options_stereo_reconstruction.cpp
)

# 実行ファイルをリンクする
target_link_libraries(stereo_reconstruction
  ktl 
  ktlrt
  ${Boost_SYSTEM_LIBRARY}
)

ament_target_dependencies(stereo_reconstruction
  "rclcpp"
  "sensor_msgs"
  "OpenCV"
  "message_filters"
  "tf2_msgs"
  "tf2_geometry_msgs"
  "rcl_interfaces"
  "rcutils"
  "pcl_conversions"

)
install(TARGETS stereo_reconstruction
  DESTINATION lib/${PROJECT_NAME}
)

# stereo_vision node
add_executable(stereo_vision
  src/stereo_vision.cpp
  option/options_stereo_vision.cpp
)
ament_target_dependencies(stereo_vision
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
  "message_filters"
)
install(TARGETS
  stereo_vision
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
