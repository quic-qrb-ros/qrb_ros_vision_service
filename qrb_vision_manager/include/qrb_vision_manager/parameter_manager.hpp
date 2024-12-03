// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#ifndef QRB_VISION_MANAGER__PARAMETER_MANAGER_HPP_
#define QRB_VISION_MANAGER__PARAMETER_MANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "qrb_vision_manager/node_info.hpp"

#define CONFIG_FILE_PATH ("/opt/qcom/qirf-sdk/etc/qrb_vision_manager/vision_manager_node.conf")

namespace qrb::vision_manager
{
class ParameterManager
{
public:
  static std::shared_ptr<ParameterManager> get_instance();

  bool get_node_parameter(const std::string & name, NodeInfo & node_info) const;

private:
  ParameterManager();

  bool init();

  std::map<std::string, NodeInfo> node_infos_map_;

  bool inited_;
};

}  // namespace qrb::vision_manager
#endif
