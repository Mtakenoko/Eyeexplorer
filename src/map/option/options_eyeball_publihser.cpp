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

#include "options_eyeball_publihser.hpp"

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
    double *x, double *y, double *z,
    double *rx, double *ry, double *rz)
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
    if (x != nullptr)
    {
      ss << " -x: Set Position X" << std::endl;
      ss << "   (default) : 0.3368" << std::endl;
    }
    if (y != nullptr)
    {
      ss << " -y: Set Position Y" << std::endl;
      ss << "   (default) : -0.1555" << std::endl;
    }
    if (z != nullptr)
    {
      ss << " -z: Set Position Z" << std::endl;
      ss << "   (default) : 0.015" << std::endl;
    }
    if (rx != nullptr)
    {
      ss << " -rx: Set Rotation X" << std::endl;
      ss << "   (default) : 0.024" << std::endl;
    }
    if (ry != nullptr)
    {
      ss << " -ry: Set Rotation Y" << std::endl;
      ss << "   (default) : 0.024" << std::endl;
    }
    if (rz != nullptr)
    {
      ss << " -rz: Set Rotation Z" << std::endl;
      ss << "   (default) : 0.024" << std::endl;
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

  auto x_str = get_command_option(args, "-x");
  if (!x_str.empty())
  {
    *x = std::stod(x_str.c_str());
  }

  auto y_str = get_command_option(args, "-y");
  if (!y_str.empty())
  {
    *y = std::stod(y_str.c_str());
  }

  auto z_str = get_command_option(args, "-z");
  if (!z_str.empty())
  {
    *z = std::stod(z_str.c_str());
  }

  auto rx_str = get_command_option(args, "-rx");
  if (!rx_str.empty())
  {
    *rx = std::stod(rx_str.c_str());
  }

  auto ry_str = get_command_option(args, "-ry");
  if (!ry_str.empty())
  {
    *ry = std::stod(ry_str.c_str());
  }

  auto rz_str = get_command_option(args, "-rz");
  if (!rz_str.empty())
  {
    *rz = std::stod(rz_str.c_str());
  }

  return true;
}
