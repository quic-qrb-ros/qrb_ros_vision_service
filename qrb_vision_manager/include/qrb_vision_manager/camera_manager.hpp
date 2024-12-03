// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#ifndef QRB_VISION_MANAGER__CAMERA_MANAGER_HPP_
#define QRB_VISION_MANAGER__CAMERA_MANAGER_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "qrb_vision_manager/node_info.hpp"

namespace qrb::vision_manager
{
struct CameraStatus
{
  uint64_t pipeline_id;
  uint8_t camera_id;
  std::string topic_name;
  uint64_t unique_id;
  int height;
  int width;
  int call_number;
};

class CameraManager
{
public:
  static std::shared_ptr<CameraManager> get_instance();

  bool camera_available(const uint8_t camera_id);

  bool can_stop_camera(const uint8_t camera_id);

  void notify_camera_start(const CameraStatus & camera);

  void notify_camera_stop(const uint8_t camera_id);

  bool get_camera_status(const uint8_t camera_id, CameraStatus & status);

private:
  CameraManager();

  std::mutex mtx_;
  std::vector<CameraStatus> cam_status_list_;
};

}  // namespace qrb::vision_manager
#endif