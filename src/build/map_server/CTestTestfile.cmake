# CMake generated Testfile for 
# Source directory: /home/takeyama/workspace/ros2_eyeexplorer/src/arm/map_server
# Build directory: /home/takeyama/workspace/ros2_eyeexplorer/src/build/map_server
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(copyright "/usr/bin/python3" "-u" "/home/takeyama/workspace/ros2_ws/install/ament_cmake_test/share/ament_cmake_test/cmake/run_test.py" "/home/takeyama/workspace/ros2_eyeexplorer/src/build/map_server/test_results/map_server/copyright.xunit.xml" "--package-name" "map_server" "--output-file" "/home/takeyama/workspace/ros2_eyeexplorer/src/build/map_server/ament_copyright/copyright.txt" "--command" "/home/takeyama/workspace/ros2_ws/install/ament_copyright/bin/ament_copyright" "--xunit-file" "/home/takeyama/workspace/ros2_eyeexplorer/src/build/map_server/test_results/map_server/copyright.xunit.xml")
set_tests_properties(copyright PROPERTIES  LABELS "copyright;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/takeyama/workspace/ros2_eyeexplorer/src/arm/map_server")
add_test(cppcheck "/usr/bin/python3" "-u" "/home/takeyama/workspace/ros2_ws/install/ament_cmake_test/share/ament_cmake_test/cmake/run_test.py" "/home/takeyama/workspace/ros2_eyeexplorer/src/build/map_server/test_results/map_server/cppcheck.xunit.xml" "--package-name" "map_server" "--output-file" "/home/takeyama/workspace/ros2_eyeexplorer/src/build/map_server/ament_cppcheck/cppcheck.txt" "--command" "/home/takeyama/workspace/ros2_ws/install/ament_cppcheck/bin/ament_cppcheck" "--xunit-file" "/home/takeyama/workspace/ros2_eyeexplorer/src/build/map_server/test_results/map_server/cppcheck.xunit.xml")
set_tests_properties(cppcheck PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/takeyama/workspace/ros2_eyeexplorer/src/arm/map_server")
add_test(cpplint "/usr/bin/python3" "-u" "/home/takeyama/workspace/ros2_ws/install/ament_cmake_test/share/ament_cmake_test/cmake/run_test.py" "/home/takeyama/workspace/ros2_eyeexplorer/src/build/map_server/test_results/map_server/cpplint.xunit.xml" "--package-name" "map_server" "--output-file" "/home/takeyama/workspace/ros2_eyeexplorer/src/build/map_server/ament_cpplint/cpplint.txt" "--command" "/home/takeyama/workspace/ros2_ws/install/ament_cpplint/bin/ament_cpplint" "--xunit-file" "/home/takeyama/workspace/ros2_eyeexplorer/src/build/map_server/test_results/map_server/cpplint.xunit.xml")
set_tests_properties(cpplint PROPERTIES  LABELS "cpplint;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/takeyama/workspace/ros2_eyeexplorer/src/arm/map_server")
add_test(lint_cmake "/usr/bin/python3" "-u" "/home/takeyama/workspace/ros2_ws/install/ament_cmake_test/share/ament_cmake_test/cmake/run_test.py" "/home/takeyama/workspace/ros2_eyeexplorer/src/build/map_server/test_results/map_server/lint_cmake.xunit.xml" "--package-name" "map_server" "--output-file" "/home/takeyama/workspace/ros2_eyeexplorer/src/build/map_server/ament_lint_cmake/lint_cmake.txt" "--command" "/home/takeyama/workspace/ros2_ws/install/ament_lint_cmake/bin/ament_lint_cmake" "--xunit-file" "/home/takeyama/workspace/ros2_eyeexplorer/src/build/map_server/test_results/map_server/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/takeyama/workspace/ros2_eyeexplorer/src/arm/map_server")
add_test(uncrustify "/usr/bin/python3" "-u" "/home/takeyama/workspace/ros2_ws/install/ament_cmake_test/share/ament_cmake_test/cmake/run_test.py" "/home/takeyama/workspace/ros2_eyeexplorer/src/build/map_server/test_results/map_server/uncrustify.xunit.xml" "--package-name" "map_server" "--output-file" "/home/takeyama/workspace/ros2_eyeexplorer/src/build/map_server/ament_uncrustify/uncrustify.txt" "--command" "/home/takeyama/workspace/ros2_ws/install/ament_uncrustify/bin/ament_uncrustify" "--xunit-file" "/home/takeyama/workspace/ros2_eyeexplorer/src/build/map_server/test_results/map_server/uncrustify.xunit.xml")
set_tests_properties(uncrustify PROPERTIES  LABELS "uncrustify;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/takeyama/workspace/ros2_eyeexplorer/src/arm/map_server")
add_test(xmllint "/usr/bin/python3" "-u" "/home/takeyama/workspace/ros2_ws/install/ament_cmake_test/share/ament_cmake_test/cmake/run_test.py" "/home/takeyama/workspace/ros2_eyeexplorer/src/build/map_server/test_results/map_server/xmllint.xunit.xml" "--package-name" "map_server" "--output-file" "/home/takeyama/workspace/ros2_eyeexplorer/src/build/map_server/ament_xmllint/xmllint.txt" "--command" "/home/takeyama/workspace/ros2_ws/install/ament_xmllint/bin/ament_xmllint" "--xunit-file" "/home/takeyama/workspace/ros2_eyeexplorer/src/build/map_server/test_results/map_server/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/takeyama/workspace/ros2_eyeexplorer/src/arm/map_server")
