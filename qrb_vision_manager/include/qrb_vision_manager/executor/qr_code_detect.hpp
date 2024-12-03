// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#ifndef QRB_VISION_MANAGER__EXECUTOR__QR_CODE_DETECT_HPP_
#define QRB_VISION_MANAGER__EXECUTOR__QR_CODE_DETECT_HPP_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "opencv2/opencv.hpp"
#include "qrb_vision_manager/executor/executor_base.hpp"

namespace qrb::vision_manager
{
class QRCodeDetectExecutor : public ExecutorBase
{
public:
  QRCodeDetectExecutor(std::shared_ptr<CallbackManager> & cb_manager, const uint8_t camera_id);

  QRCodeDetectExecutor(std::shared_ptr<CallbackManager> & cb_manager,
      const std::string & camera_name);
  ~QRCodeDetectExecutor();

  bool execute(const std::vector<std::string> & files);

  bool execute(const uint8_t camera_id, CameraStatus & cam_status);

  bool execute();

  bool stop_case(bool stop_camera = false);

protected:
  void detect_image(cv::Mat & image);

  bool has_contain(const std::string contain);

private:
  bool load_image(const std::string & path, cv::Mat & mat);

  bool is_stream_ = false;
  std::string camera_name_;
  std::vector<std::string> contains_;
  bool running_{ false };
  std::mutex mtx_;
  std::condition_variable cv_;
  std::shared_ptr<std::thread> thread_;
};

}  // namespace qrb::vision_manager
#endif