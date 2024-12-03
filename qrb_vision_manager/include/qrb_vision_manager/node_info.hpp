// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#ifndef QRB_VISION_MANAGER__NODE_INFO_HPP_
#define QRB_VISION_MANAGER__NODE_INFO_HPP_

#include <algorithm>
#include <map>
#include <string>
#include <vector>

namespace qrb::vision_manager
{
struct NodeInfo
{
  std::string package_name;
  std::string plugin_name;
  std::string node_name;
  std::vector<std::string> pub_topic_name;
  std::vector<std::string> sub_topic_name;

  // map 's vector format:
  // "type_name, name, type, value1, value2"
  std::map<std::string, std::vector<std::vector<std::string>>> parameter_list;

  std::vector<std::string> parameter_name_list;
};

struct ComponentInfo
{
  std::string package_name;
  std::string plugin_name;
  std::string node_name;
  std::vector<std::string> pub_topic_name;  // format: topicA_name, topicA_type ...
  std::vector<std::string> sub_topic_name;
  std::vector<std::string> parameters;  // format: "name,type,value"
  bool multi_topic;
};

struct ComponentStatus
{
  uint64_t pipeline_id;
  std::string node_name;
  std::string topic_name;
  uint64_t unique_id;
  int height;
  int width;
  int call_number;
};

}  // namespace qrb::vision_manager
#endif