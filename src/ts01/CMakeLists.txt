cmake_minimum_required(VERSION 3.5)
project(ts01)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

#ts01_sensor node
add_executable(ts01_sensor
  src/ts01_sensor.cpp
)
target_link_libraries(ts01_sensor -lts01 -lktl)
ament_target_dependencies(ts01_sensor
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)
install(TARGETS
  ts01_sensor
  DESTINATION lib/${PROJECT_NAME}
)

#ts01_manager node
add_executable(ts01_manager
  src/ts01_manager.cpp
  src/manage.cpp
  src/TS_01.cpp
)
target_link_libraries(ts01_manager -lts01 -lktl)
ament_target_dependencies(ts01_manager
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)
install(TARGETS
ts01_manager
  DESTINATION lib/${PROJECT_NAME}
)

#ts01_AI_listener node
add_executable(ts01_AI_listener
  src/ts01_AI_listener.cpp
)
target_link_libraries(ts01_AI_listener)
ament_target_dependencies(ts01_AI_listener
  "rclcpp"
  "std_msgs"
)
install(TARGETS
  ts01_AI_listener
  DESTINATION lib/${PROJECT_NAME}
)

#ts01_DI_listener node
add_executable(ts01_DI_listener
  src/ts01_DI_listener.cpp
)
target_link_libraries(ts01_DI_listener)
ament_target_dependencies(ts01_DI_listener
  "rclcpp"
  "std_msgs"
)
install(TARGETS
  ts01_DI_listener
  DESTINATION lib/${PROJECT_NAME}
)

#ts01_AO_talker node
add_executable(ts01_AO_talker
  src/ts01_AO_talker.cpp
)
target_link_libraries(ts01_AO_talker)
ament_target_dependencies(ts01_AO_talker
  "rclcpp"
  "std_msgs"
)
install(TARGETS
  ts01_AO_talker
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
