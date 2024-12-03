// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#include "qrb_vision_manager/parameter_manager.hpp"

#include <fstream>
#include <iostream>
#include <mutex>

#include "yaml-cpp/yaml.h"

using namespace std;

namespace qrb::vision_manager
{
static shared_ptr<ParameterManager> singleton = nullptr;
static once_flag singleton_flag;

shared_ptr<ParameterManager> ParameterManager::get_instance()
{
  if (singleton == nullptr) {
    singleton = shared_ptr<ParameterManager>(new ParameterManager());
  }

  return singleton;
}

ParameterManager::ParameterManager()
{
  inited_ = init();
}

bool ParameterManager::get_node_parameter(const string & name, NodeInfo & node_info) const
{
  if (!inited_) {
    return false;
  }
  auto iter = node_infos_map_.find(name);
  if (iter != node_infos_map_.end()) {
    node_info = (*iter).second;
    return true;
  }
  cout << "can not find this name: " << name << endl;
  return false;
}

bool ParameterManager::init()
{
  YAML::Node config = YAML::LoadFile(CONFIG_FILE_PATH);
  if (!config) {
    cout << "there is no vision_manager_node conf file, please check it" << endl;
    cout << "file path: " << CONFIG_FILE_PATH << endl;
    return false;
  }

  for (auto it = config["package_list"].begin(); it != config["package_list"].end(); ++it) {
    NodeInfo info;
    info.node_name = it->as<string>();
    YAML::Node node = config[info.node_name];
    info.package_name = node["packages"].as<string>();
    info.plugin_name = node["plugin"].as<string>();

    if (node["pub_topic"] && !node["pub_topic"].IsNull()) {
      auto topics = node["pub_topic"].as<vector<string>>();
      info.pub_topic_name.resize(topics.size());
      copy(topics.begin(), topics.end(), info.pub_topic_name.begin());
    }
    if (node["sub_topic"] && !node["sub_topic"].IsNull()) {
      auto topics = node["sub_topic"].as<vector<string>>();
      info.sub_topic_name.resize(topics.size());
      copy(topics.begin(), topics.end(), info.sub_topic_name.begin());
    }

    if (!node["parameter_list"] || node["parameter_list"].IsNull()) {
      cout << "parameter_list in " << info.node_name << " is null" << endl;
      cout << "push name in map: " << info.node_name << endl;
      node_infos_map_.insert(make_pair(info.node_name, info));
      continue;
    }

    for (auto i = node["parameter_list"].begin(); i != node["parameter_list"].end(); ++i) {
      string type_name = i->first.as<string>();
      vector<vector<string>> param_list;
      for (auto j = node["parameter_list"][type_name].begin();
           j != node["parameter_list"][type_name].end(); ++j) {
        vector<string> param;
        string param_name = j->first.as<string>();
        auto list = j->second.as<vector<string>>();
        param.push_back(param_name);
        param.resize(list.size() + 1);
        copy(list.begin(), list.end(), param.begin() + 1);
        param_list.push_back(param);
        if (i == node["parameter_list"].begin()) {
          info.parameter_name_list.push_back(param_name);
        }
      }
      cout << "push name in param: " << type_name << endl;
      info.parameter_list.insert(make_pair(type_name, param_list));
    }
    cout << "push name in map: " << info.node_name << endl;
    node_infos_map_.insert(make_pair(info.node_name, info));
  }

  return true;
}

}  // namespace qrb::vision_manager
