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

#include "options_insertpoint.hpp"

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
    float *param,
    size_t * save)
{
  std::vector<std::string> args(argv, argv + argc);

  if (find_command_option(args, "-h"))
  {
    std::stringstream ss;
    ss << "Usage:" << std::endl;
    ss << " -h: This message." << std::endl;    
    if (param != nullptr)
    {
      ss << " -p: Set Insertion point algo Param" << std::endl;
      ss << "   (default) : 1000.0" << std::endl;
    }
    if (save != nullptr) {
      ss << " -s: Save File:" << std::endl;
      ss << "    0 - Do not save the insertpoint file" << std::endl;
      ss << "    1 - Ssave the insertpoint file" << std::endl;
    }
    std::cout << ss.str();
    return false;
  }

  auto param_str = get_command_option(args, "-p");
  if (!param_str.empty())
  {
    *param = std::stof(param_str.c_str());
  }

  auto save_str = get_command_option(args, "-s");
  if (!save_str.empty()) {
    *save = std::stoul(save_str.c_str());
  }


  return true;
}
