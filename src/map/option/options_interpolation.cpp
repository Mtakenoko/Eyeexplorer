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

#include "options_interpolation.hpp"

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
    int *num_cut, double *thresh_distance)
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
    if (num_cut != nullptr)
    {
      ss << " --cut: Set Cube pointclouds resolution  " << std::endl;
      ss << "   (default) : 10" << std::endl;
    }
    if (thresh_distance != nullptr)
    {
      ss << " --dis: Set Distance between cube point and map" << std::endl;
      ss << "   (default) : 0.003" << std::endl;
    }
    std::cout << ss.str();
    return false;
  }

  auto depth_str = get_command_option(args, "-d");
  if (!depth_str.empty())
  {
    *depth = std::stoul(depth_str.c_str());
  }

  auto reliability_str = get_command_option(args, "-r");
  if (!reliability_str.empty())
  {
    unsigned int r = std::stoul(reliability_str.c_str());
    *reliability_policy =
        r ? RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  }

  auto history_str = get_command_option(args, "-k");
  if (!history_str.empty())
  {
    unsigned int r = std::stoul(history_str.c_str());
    *history_policy = r ? RMW_QOS_POLICY_HISTORY_KEEP_ALL : RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  }

  auto num_cut_str = get_command_option(args, "--cut");
  if (!num_cut_str.empty())
  {
    *num_cut = std::stoul(num_cut_str.c_str());
  }

  auto thresh_distance_str = get_command_option(args, "--dis");
  if (!thresh_distance_str.empty())
  {
    *thresh_distance = std::stod(thresh_distance_str.c_str());
  }


  return true;
}
