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

#include "options_reconstructor.hpp"

bool find_command_option(const std::vector<std::string> &args, const std::string &option)
{
  return std::find(args.begin(), args.end(), option) != args.end();
}

std::string get_command_option(const std::vector<std::string> &args, const std::string &option)
{
  auto it = std::find(args.begin(), args.end(), option);
  if (it != args.end() && ++it != args.end())
  {
    return *it;
  }
  return std::string();
}

bool get_flag_option(const std::vector<std::string> &args, const std::string &option)
{
  auto it = std::find(args.begin(), args.end(), option);
  if (it != args.end())
  {
    return true;
  }
  return false;
}

bool parse_command_options(
    int argc, char **argv,
    size_t *depth,
    rmw_qos_reliability_policy_t *reliability_policy,
    rmw_qos_history_policy_t *history_policy,
    bool *show_camera, bool *ceres_stdout,
    size_t *mode,
    bool *est_move, float *thresh_knn_ratio, float *thresh_ransac,
    int *cpu_core, size_t *scene,
    size_t *matching, size_t *extractor, size_t *publish)
{
  std::vector<std::string> args(argv, argv + argc);

  if (find_command_option(args, "-h"))
  {
    std::stringstream ss;
    ss << "Usage:" << std::endl;
    ss << " -h: This message." << std::endl;
    ss << " -r: Reliability QoS setting:" << std::endl;
    ss << "    0 - best effort" << std::endl;
    ss << "    1 - reliable (default)" << std::endl;
    ss << " -d: Depth of the queue: only honored if used together with 'keep last'. "
       << "10 (default)" << std::endl;
    ss << " -k: History QoS setting:" << std::endl;
    ss << "    0 - only store up to N samples, configurable via the queue depth (default)" << std::endl;
    ss << "    1 - keep all the samples" << std::endl;
    if (show_camera != nullptr)
    {
      ss << " --show: Show Matching scene:" << std::endl;
      ss << "    0 - Do not show the camera stream (default)" << std::endl;
      ss << "    1 - Show the camera stream" << std::endl;
    }
    if (ceres_stdout != nullptr)
    {
      ss << " --ceres-stdout: Show Matching scene:" << std::endl;
      ss << "    0 - Do not show (default)" << std::endl;
      ss << "    1 - Show the ceres stdout" << std::endl;
    }
    if (mode != nullptr)
    {
      ss << " --mode: Select Mode:" << std::endl;
      ss << "    0 - Normal (default)" << std::endl;
      ss << "    1 - Eye" << std::endl;
    }
    if (est_move != nullptr)
    {
      ss << " --est-move: Estimation Movement.  " << std::endl;
      ss << "    0 - OFF (default)" << std::endl;
      ss << "    1 - ON" << std::endl;
    }
    if (thresh_knn_ratio != nullptr)
    {
      ss << " --knn-ratio: Set Threshhold of knn matching ratio  " << std::endl;
      ss << "   (default) : 0.7f" << std::endl;
    }
    if (thresh_ransac != nullptr)
    {
      ss << " --thresh-ransac: Set Threshhold of RANSAC used for   " << std::endl;
      ss << "   (default) : 5.0" << std::endl;
    }
    if (cpu_core != nullptr)
    {
      ss << " --core: Set Number of CPU core used for Bundler" << std::endl;
      ss << "   (default) : 8 cores" << std::endl;
    }
    if (scene != nullptr)
    {
      ss << " --scene: Set Number of Scenes used for triangulation" << std::endl;
      ss << "   (default) : 4 scenes" << std::endl;
    }
    if (matching != nullptr)
    {
      ss << " --match: Set Matching method" << std::endl;
      ss << "   KNN          : 0" << std::endl;
      ss << "   BruteForce   : 1  (default)" << std::endl;
    }
    if (matching != nullptr)
    {
      ss << " --extractor: Set Extractor method" << std::endl;
      ss << "   ORB    : 0" << std::endl;
      ss << "   AKAZE  : 1  (default)" << std::endl;
      ss << "   BRISK  : 2 " << std::endl;
      ss << "   SIFT   : 3 " << std::endl;
      ss << "   SURF   : 4 " << std::endl;
      ss << "   BRIEF  : 5 " << std::endl;
    }
    if (publish != nullptr)
    {
      ss << " --publish: Set Publish pointcloud" << std::endl;
      ss << "   NORMAL        : 0 " << std::endl;
      ss << "   NORMAL_HOLD   : 1 " << std::endl;
      ss << "   BUNDLE        : 2 " << std::endl;
      ss << "   BUNDLE_HOLD   : 3 " << std::endl;
      ss << "   FILTER        : 4 " << std::endl;
      ss << "   FILTER_HOLD   : 5 (default)" << std::endl;
      ss << "   ESTIMATE      : 6 " << std::endl;
      ss << "   ESTIMATE_HOLD : 7 " << std::endl;
    }

    std::cout << ss.str();
    return false;
  }

  auto reliability_str = get_command_option(args, "-r");
  if (!reliability_str.empty())
  {
    unsigned int r = std::stoul(reliability_str.c_str());
    *reliability_policy =
        r ? RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  }

  auto depth_str = get_command_option(args, "-d");
  if (!depth_str.empty())
  {
    *depth = std::stoul(depth_str.c_str());
  }

  auto history_str = get_command_option(args, "-k");
  if (!history_str.empty())
  {
    unsigned int r = std::stoul(history_str.c_str());
    *history_policy = r ? RMW_QOS_POLICY_HISTORY_KEEP_ALL : RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  }

  auto show_camera_str = get_command_option(args, "--show");
  if (!show_camera_str.empty())
  {
    *show_camera = std::stoul(show_camera_str.c_str()) != 0 ? true : false;
  }

  auto ceres_stdout_str = get_command_option(args, "--ceres-stdout");
  if (!ceres_stdout_str.empty())
  {
    *ceres_stdout = std::stoul(ceres_stdout_str.c_str()) != 0 ? true : false;
  }

  auto mode_str = get_command_option(args, "--mode");
  if (!mode_str.empty())
  {
    *mode = std::stoul(mode_str.c_str());
  }

  auto est_move_str = get_command_option(args, "--est-move");
  if (!est_move_str.empty())
  {
    *est_move = std::stoul(est_move_str.c_str()) != 0 ? true : false;
  }

  auto thresh_knn_ratio_str = get_command_option(args, "--knn-ratio");
  if (!thresh_knn_ratio_str.empty())
  {
    *thresh_knn_ratio = std::stoul(thresh_knn_ratio_str.c_str());
  }

  auto thresh_ransac_str = get_command_option(args, "--thresh-ransac");
  if (!thresh_ransac_str.empty())
  {
    *thresh_ransac = std::stoul(thresh_ransac_str.c_str());
  }

  auto cpu_core_str = get_command_option(args, "--core");
  if (!cpu_core_str.empty())
  {
    *cpu_core = std::stoul(cpu_core_str.c_str());
  }

  auto scene_str = get_command_option(args, "--scene");
  if (!scene_str.empty())
  {
    *scene = std::stoul(scene_str.c_str());
  }

  auto matching_str = get_command_option(args, "--match");
  if (!matching_str.empty())
  {
    *matching = std::stoul(matching_str.c_str());
  }

  auto extractor_str = get_command_option(args, "--extractor");
  if (!extractor_str.empty())
  {
    *extractor = std::stoul(extractor_str.c_str());
  }

  auto publish_str = get_command_option(args, "--publish");
  if (!publish_str.empty())
  {
    *publish = std::stoul(publish_str.c_str());
  }

  return true;
}
