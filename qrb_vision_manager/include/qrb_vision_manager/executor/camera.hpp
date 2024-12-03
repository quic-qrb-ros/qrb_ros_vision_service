// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#ifndef QRB_VISION_MANAGER__EXECUTOR__CAMERA_EXECUTOR_HPP_
#define QRB_VISION_MANAGER__EXECUTOR__CAMERA_EXECUTOR_HPP_

#include <string>
#include <vector>

#include "qrb_vision_manager/executor/executor_base.hpp"

namespace qrb::vision_manager
{
class CameraExecutor : public ExecutorBase
{
public:
  CameraExecutor(std::shared_ptr<CallbackManager> & cb_manager, const uint8_t camera_id);
  ~CameraExecutor();

  bool enable(int height, int width, CameraStatus & cam_status) override;

  bool stop(int height = 0, int width = 0, bool stop_camera = false) override;

  void add_new_listener(int height, int width) override;

protected:
  const std::string get_topic_name(int height, int width) override;
};

}  // namespace qrb::vision_manager
#endif