// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "../include/options_reconstruction_online_tf2.hpp"

bool find_command_option(const std::vector<std::string> & args, const std::string & option)
{
  return std::find(args.begin(), args.end(), option) != args.end();
}

std::string get_command_option(const std::vector<std::string> & args, const std::string & option)
{
  auto it = std::find(args.begin(), args.end(), option);
  if (it != args.end() && ++it != args.end()) {
    return *it;
  }
  return std::string();
}

bool get_flag_option(const std::vector<std::string> & args, const std::string & option)
{
  auto it = std::find(args.begin(), args.end(), option);
  if (it != args.end()) {
    return true;
  }
  return false;
}

bool parse_command_options(
  int argc, char ** argv, 
  size_t * depth,
  rmw_qos_reliability_policy_t * reliability_policy,
  rmw_qos_history_policy_t * history_policy, 
  bool * show_camera,
  size_t * feature, size_t * match, size_t * prjMat)
{
  std::vector<std::string> args(argv, argv + argc);

  if (find_command_option(args, "-h")) {
    std::stringstream ss;
    ss << "Usage:" << std::endl;
    ss << " -h: This message." << std::endl;
    ss << " -r: Reliability QoS setting:" << std::endl;
    ss << "    0 - best effort" << std::endl;
    ss << "    1 - reliable (default)" << std::endl;
    ss << " -d: Depth of the queue: only honored if used together with 'keep last'. " <<
      "10 (default)" << std::endl;
    ss << " -k: History QoS setting:" << std::endl;
    ss << "    0 - only store up to N samples, configurable via the queue depth (default)" <<
      std::endl;
    ss << "    1 - keep all the samples" << std::endl;
    if (show_camera != nullptr) {
      ss << " -s: Camera stream:" << std::endl;
      ss << "    0 - Do not show the camera stream (default)" << std::endl;
      ss << "    1 - Show the camera stream" << std::endl;
    }
    if (feature != nullptr) {
      ss << " -e: Feature point detector.  " << std::endl;
      ss << "   0 - AKAZE (default)" << std::endl;
      ss << "   1 - ORB" << std::endl;
      ss << "   2 - BRISK" << std::endl;
    }
    if (match != nullptr) {
      ss << " -m: Matching Feature point.  " << std::endl;
      ss << "   0 - Brute-Force matcher (default)" << std::endl;
      ss << "   1 - FLANN" << std::endl;
    }
    if (prjMat != nullptr) {
      ss << " -p: Projection Matrix  " << std::endl;
      ss << "   0 - 5点アルゴリズム " << std::endl;
      ss << "   1 - 運動学(default)" << std::endl;
    }
    std::cout << ss.str();
    return false;
  }

  auto show_camera_str = get_command_option(args, "-s");
  if (!show_camera_str.empty()) {
    *show_camera = std::stoul(show_camera_str.c_str()) != 0 ? true : false;
  }

  auto depth_str = get_command_option(args, "-d");
  if (!depth_str.empty()) {
    *depth = std::stoul(depth_str.c_str());
  }

  auto reliability_str = get_command_option(args, "-r");
  if (!reliability_str.empty()) {
    unsigned int r = std::stoul(reliability_str.c_str());
    *reliability_policy =
      r ? RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  }

  auto history_str = get_command_option(args, "-k");
  if (!history_str.empty()) {
    unsigned int r = std::stoul(history_str.c_str());
    *history_policy = r ? RMW_QOS_POLICY_HISTORY_KEEP_ALL : RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  }

  auto feature_str = get_command_option(args, "-e");
  if (!feature_str.empty()) {
    *feature = std::stoul(feature_str.c_str());
  }

  auto match_str = get_command_option(args, "-m");
  if (!match_str.empty()) {
    if (!match_str.empty()) {
      *match = std::stoul(match_str.c_str());
    }
  }

  auto prjMat_str = get_command_option(args, "-p");
  if (!prjMat_str.empty()) {
    *prjMat = std::stoul(prjMat_str.c_str());  
  }

  return true;
}
