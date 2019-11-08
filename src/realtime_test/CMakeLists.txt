###############################################
#
#   Author : Yura Aoyama
#   Description : Describe build procedure
#
##############################################

cmake_minimum_required(VERSION 3.5)
project(realtime_test)  # パッケージ名

# C++14を利用
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# すべてのワーニングを表示する
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# link_directories(/usr/local/lib) #ライブラリディレクトリ指定 
# set(CMAKE_CXX_FLAGS "-std=c++11 -lts01 -lktl -lktlrt -pthread")

# 依存パッケージを探す
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rttest)
find_package(tlsf_cpp)
if($ENV{ROS_DISTRO} STREQUAL dashing)
  find_package(rclcpp_components REQUIRED)
endif()
# find_package(hello_world_msgs REQUIRED)

include_directories(include)

# パッケージの存在確認
if(NOT rttest_FOUND)
  message(STATUS "rttest not found. realtime_test package will not be built.")
  return()
endif()

# ビルド設定の便利関数
function(custom_executable target)
  add_executable(${target} src/${target}.cpp)                 # スタンドアローンノードの実行ファイルをコンパイルする
  target_link_libraries(${target} ts01 ktl ktlrt)             # 実行ファイルをリンクする
  ament_target_dependencies(${target} "rclcpp" "rclcpp_components" "std_msgs" "rttest" "tlsf_cpp")    # コンパイルターゲットに依存パッケージの情報を追加する
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})  # 実行ファイルをインストール
endfunction()

# talkerノードのビルド設定
# custom_executable(send_pulse_component)
# listenerノードのビルド設定
# custom_executable(listener)

custom_executable(realtime_exec)


if($ENV{ROS_DISTRO} STREQUAL dashing)
  # 共有ライブラリの生成
  add_library(send_pulse_component SHARED
    src/send_pulse_component.cpp)
  target_compile_definitions(send_pulse_component
    PRIVATE "COMPOSITION_BUILDING_DLL")
  ament_target_dependencies(send_pulse_component
    "rclcpp"
    "rclcpp_components"
    "std_msgs"
    "rttest"
    "tlsf_cpp")
  # クラスローダーへの共有ライブラリの登録
  rclcpp_components_register_nodes(send_pulse_component
    "realtime_test::SendPulseComponent")

  # 共有ライブラリの生成
  add_library(ts01_outp_component SHARED
    src/ts01_outp_component.cpp)
  target_compile_definitions(ts01_outp_component
    PRIVATE "COMPOSITION_BUILDING_DLL")
  ament_target_dependencies(ts01_outp_component
    "rclcpp"
    "rclcpp_components"
    "std_msgs"
    "rttest"
    "tlsf_cpp")
  # クラスローダーへの共有ライブラリの登録
  rclcpp_components_register_nodes(ts01_outp_component
    "realtime_test::Ts01OutpComponent")
endif()

install(DIRECTORY
#     launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
