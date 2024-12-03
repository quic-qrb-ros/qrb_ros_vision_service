// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#ifndef QRB_VISION_MANAGER__TASK_MANAGER_HPP_
#define QRB_VISION_MANAGER__TASK_MANAGER_HPP_

#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <unordered_map>

#include "qrb_vision_manager/callback_manager.hpp"
#include "qrb_vision_manager/executor/executor_base.hpp"

namespace qrb::vision_manager
{
class MissionManager
{
public:
  MissionManager(std::shared_ptr<CallbackManager> & cb_manager);
  ~MissionManager(){};

  void start_task(const int type, const uint8_t camera_id, const int height, const int width);
  void stop_task(const int type, const uint8_t camera_id, const int height, const int width);

  void start_qr_case(const int type, const std::vector<std::string> & names);
  void start_qr_case(const int type, const uint8_t camera_id);
  void start_qr_case(const int type, const std::string & name);

  bool stop_qr_case(const int type, const uint8_t camera_id);
  bool stop_qr_case(const int type, const std::string & name);

private:
  void remove_executor(const std::string & name);
  std::shared_ptr<ExecutorBase> check_executor_exist(const std::string & name);

  std::shared_ptr<CallbackManager> cb_manager_;
  std::shared_ptr<CameraManager> camera_manager_;
  std::shared_ptr<ParameterManager> param_manager_;
  std::unordered_map<std::string, std::shared_ptr<ExecutorBase>> executor_maps_;

  std::mutex mtx_;
};

}  // namespace qrb::vision_manager
#endif