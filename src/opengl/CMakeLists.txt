cmake_minimum_required(VERSION 3.5)
project(opengl)

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
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

# opengl_test node
# スタンドアローンノードの実行ファイルをコンパイルする
add_executable(opengl_test
  src/opengl_test.cpp
)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )
target_link_libraries(opengl_test ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )
# コンパイルターゲットに依存パッケージの情報を追加する
ament_target_dependencies(opengl_test
  "rclcpp"
  "OpenGL"
  "GLUT"
)
# 実行ファイルをインストール
install(TARGETS
  opengl_test
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
