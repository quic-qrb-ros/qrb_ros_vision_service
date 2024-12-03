// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#ifndef QRB_VISION_MANAGER__REQUEST_MANAGER_HPP_
#define QRB_VISION_MANAGER__REQUEST_MANAGER_HPP_

#include <iostream>
#include <string>
#include <vector>

#include "qrb_vision_manager/callback_manager.hpp"
#include "qrb_vision_manager/mission_manager.hpp"

namespace qrb::vision_manager
{
class RequestManager
{
public:
  RequestManager();

  void execute_task(const int type, const uint8_t camera_id, const int height, const int width);
  bool execute_qr_case(const int type, const std::vector<std::string> & file_path);
  bool execute_qr_case(const int type, const std::string & name);
  bool execute_qr_case(const int type, const uint8_t camera_id);

  void register_qrcode_result_callback(QRCodeResultCallback cb);
  void register_qrcode_feedback_callback(QRCodeFeedbackCallback cb);
  void register_task_result_callback(TaskResultCallback cb);
  void register_pipeline_request_callback(PipelineRequestCallback cb);

  void register_create_image_sub_callback(CreateImageSubCallback cb);

private:
  std::shared_ptr<CallbackManager> cb_manager_{ nullptr };
  std::shared_ptr<MissionManager> mission_manager_{ nullptr };
};

}  // namespace qrb::vision_manager
#endif