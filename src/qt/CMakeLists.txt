cmake_minimum_required(VERSION 3.5)
project(qt)

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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

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
#demo node
add_executable(demo
  src/demo/main.cpp
  src/demo/mainwindow.cpp
)
target_link_libraries(demo
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)
# 実行ファイルをインストール
install(TARGETS demo
  DESTINATION lib/${PROJECT_NAME}
)

#qt_basic1 node
add_executable(basic1
  src/qt_basic1.cpp
)
target_link_libraries(basic1
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)
# 実行ファイルをインストール
install(TARGETS basic1
  DESTINATION lib/${PROJECT_NAME}
)

#qt_basic2 node
add_executable(basic2
  src/qt_basic2.cpp
)
target_link_libraries(basic2
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)
# 実行ファイルをインストール
install(TARGETS basic2
  DESTINATION lib/${PROJECT_NAME}
)

#qt_basic3 node
add_executable(basic3
  src/qt_basic3.cpp
)
target_link_libraries(basic3
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)
# 実行ファイルをインストール
install(TARGETS basic3
  DESTINATION lib/${PROJECT_NAME}
)

#qt_basic4 node
add_executable(basic4
  src/qt_basic4.cpp
  src/qt_mydialog.cpp
)
target_link_libraries(basic4
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)
# 実行ファイルをインストール
install(TARGETS basic4
  DESTINATION lib/${PROJECT_NAME}
)

#qt_talker node
add_executable(qt_talker
  src/qt_talker.cpp
  src/qt_talker_class.cpp
)
ament_target_dependencies(qt_talker
  "rclcpp"
  "std_msgs"
)
target_link_libraries(qt_talker
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)
# 実行ファイルをインストール
install(TARGETS qt_talker
  DESTINATION lib/${PROJECT_NAME}
)

#qt_listener node
add_executable(qt_listener
  src/qt_listener.cpp
  src/qt_listener_class.cpp
)
ament_target_dependencies(qt_listener
  "rclcpp"
  "std_msgs"
)
target_link_libraries(qt_listener
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)
# 実行ファイルをインストール
install(TARGETS qt_listener
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
