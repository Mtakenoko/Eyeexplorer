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

#include "options_gridmap.hpp"

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
    double *clampingThresMax,
    double *clampingThresMin,
    double *occupancyThres,
    double *probHit,
    double *probMiss,
    double *resolution)
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
    if (clampingThresMax != nullptr)
    {
      ss << " --clamp-max: sets the maximum threshold for occupancy clamping (sensor model)" << std::endl;
      ss << "   (default) : 0.971" << std::endl;
    }
    if (clampingThresMin != nullptr)
    {
      ss << " --clamp-min: sets the minimum threshold for occupancy clamping (sensor model)" << std::endl;
      ss << "   (default) : 0.1192" << std::endl;
    }
    if (occupancyThres != nullptr)
    {
      ss << " --occ: sets the threshold for occupancy (sensor model)" << std::endl;
      ss << "   (default) : 0.5" << std::endl;
    }
    if (probHit != nullptr)
    {
      ss << " --prob-hit: sets the probability for a -hit- (will be converted to logodds) - sensor model" << std::endl;
      ss << "   (default) : 0.7" << std::endl;
    }
    if (probMiss != nullptr)
    {
      ss << " --prob-miss: sets the probability for a -miss- (will be converted to logodds) - sensor model" << std::endl;
      ss << "   (default) : 0.4" << std::endl;
    }
    if (resolution != nullptr)
    {
      ss << " --resol: Change the resolution of the octree, scaling all voxels." << std::endl
         << "          This will not preserve the (metric) scale!" << std::endl;
      ss << "   (default) : 0.001" << std::endl;
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

  auto clampingThresMax_str = get_command_option(args, "--clamp-max");
  if (!clampingThresMax_str.empty())
  {
    *clampingThresMax = std::stod(clampingThresMax_str.c_str());
  }

  auto clampingThresMin_str = get_command_option(args, "--clamp-min");
  if (!clampingThresMin_str.empty())
  {
    *clampingThresMin = std::stod(clampingThresMin_str.c_str());
  }

  auto occupancyThres_str = get_command_option(args, "--occ");
  if (!occupancyThres_str.empty())
  {
    *occupancyThres = std::stod(occupancyThres_str.c_str());
  }

  auto probHit_str = get_command_option(args, "--prob-hit");
  if (!probHit_str.empty())
  {
    *probHit = std::stod(probHit_str.c_str());
  }

  auto probMiss_str = get_command_option(args, "--prob-miss");
  if (!probMiss_str.empty())
  {
    *probMiss = std::stod(probMiss_str.c_str());
  }

  auto resolution_str = get_command_option(args, "--resol");
  if (!resolution_str.empty())
  {
    *resolution = std::stod(resolution_str.c_str());
  }

  return true;
}
