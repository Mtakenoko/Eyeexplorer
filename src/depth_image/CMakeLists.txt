cmake_minimum_required(VERSION 3.5)
project(depth_image)

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
find_package(rcl_interfaces REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

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

################  node setting  ###############
# depth_image_creator node
add_executable(depth_image_creator
  src/depth_image_creator.cpp
  src/create.cpp
  src/MainDialog.cpp
  src/ImageWidget.cpp
)
ament_target_dependencies(depth_image_creator
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "visualization_msgs"
  "OpenCV"
  "OpenMP"
)
target_link_libraries(depth_image_creator
  ${QT_LIBRARIES}
)
install(TARGETS depth_image_creator
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
