cmake_minimum_required(VERSION 3.5)
project(test)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(VTK 6.3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(PCL COMPONENTS common REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcutils REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)

#test_talker node
add_executable(test_talker
  src/test_talker.cpp
  src/test_talker_main.cpp
)
ament_target_dependencies(test_talker
  "rclcpp"
  "std_msgs"
)
install(TARGETS
  test_talker
  DESTINATION lib/${PROJECT_NAME}
)

#test_talker node
add_executable(timer_test
  src/test_minimal_timer.cpp
)
ament_target_dependencies(timer_test
  rclcpp
)
install(TARGETS
  timer_test
  DESTINATION lib/${PROJECT_NAME}
)

#test_tf2_listener node
add_executable(test_tf2_listener
  src/test_tf2_listener.cpp
)
ament_target_dependencies(test_tf2_listener
  "rclcpp"
  "geometry_msgs"
  "tf2_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
)
install(TARGETS
  test_tf2_listener
  DESTINATION lib/${PROJECT_NAME}
)

#pcl_test node
add_executable(pcl_test
  src/pcl_test.cpp
)
# include_directories(pcl_test
#   ${PCL_INCLUDE_DIRS}
# )
# link_directories(pcl_test
#   ${PCL_LIBRARY_DIRS}
# )
# add_definitions(pcl_test
#   ${PCL_DEFINITIONS}
# )
target_link_libraries(pcl_test
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
  ${PCL_LIBRARIES}
)
ament_target_dependencies(pcl_test
  "rclcpp"
  "sensor_msgs"
  "pcl_conversions"
  "VTK 6.3"
  "Eigen3"
  "PCL 1.8"
  "PCL COMPONENTS common"
)
install(TARGETS
  pcl_test
  DESTINATION lib/${PROJECT_NAME}
)


#grid_map_test node
add_executable(grid_map_test
  src/grid_map_test.cpp
)
target_link_libraries(grid_map_test
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
  ${PCL_LIBRARIES}
)
ament_target_dependencies(grid_map_test
  "rclcpp"
  "sensor_msgs"
  "nav_msgs"
  "pcl_conversions"
  "VTK 6.3"
  "Eigen3"
  "PCL 1.8"
  "PCL COMPONENTS common"
)
install(TARGETS
grid_map_test
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
