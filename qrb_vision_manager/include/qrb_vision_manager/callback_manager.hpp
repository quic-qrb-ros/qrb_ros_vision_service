// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#ifndef QRB_VISION_MANAGER__CALLBACK_MANAGER_HPP_
#define QRB_VISION_MANAGER__CALLBACK_MANAGER_HPP_

#include <functional>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>

#include "qrb_vision_manager/node_info.hpp"

namespace qrb::vision_manager
{
using QRCodeResultCallback = std::function<void(const std::vector<std::string> & contents,
    const std::string & err_info,
    bool success,
    const std::string & camera_name)>;
using QRCodeFeedbackCallback =
    std::function<void(const std::vector<std::string> & content, const std::string & camera_name)>;
using PipelineRequestCallback = std::function<bool(int action,
    int pipeline_id,
    const std::vector<ComponentInfo> & infos,
    const std::vector<int> & ids,
    std::string & topic_name,
    int & pipeline)>;
using TaskResultCallback = std::function<void(const std::vector<std::string> & topic_name,
    const std::vector<std::string> & node_name,
    const std::string & error_info,
    bool success)>;
using ImageCallback = std::function<void(cv::Mat & image)>;
using CreateImageSubCallback =
    std::function<void(const std::string & source, const std::string & name, ImageCallback cb)>;

class CallbackManager
{
public:
  void send_qrcode_result(const std::vector<std::string> & contents,
      const std::string & err_info,
      bool success,
      const std::string & camera_name)
  {
    qrcode_result_cb_(contents, err_info, success, camera_name);
  }

  void send_qrcode_feedback(const std::vector<std::string> & contents,
      const std::string & camera_name)
  {
    qrcode_feedback_cb_(contents, camera_name);
  }

  bool send_pipeline_request(int action,
      int pipeline_id,
      const std::vector<ComponentInfo> & infos,
      const std::vector<int> & ids,
      std::string & topic_name,
      int & pipeline)
  {
    return pipeline_request_cb_(action, pipeline_id, infos, ids, topic_name, pipeline);
  }

  void send_task_result(const std::vector<std::string> & topic_name,
      const std::vector<std::string> & node_name,
      const std::string & err_info,
      bool success)
  {
    task_result_cb_(topic_name, node_name, err_info, success);
  }

  void send_create_sub_image(const std::string & source,
      const std::string & topic_name,
      ImageCallback cb)
  {
    create_image_sub_cb_(source, topic_name, cb);
  }

  void set_qrcode_result_callback(QRCodeResultCallback cb) { qrcode_result_cb_ = cb; }
  void set_qrcode_feedback_callback(QRCodeFeedbackCallback cb) { qrcode_feedback_cb_ = cb; }
  void set_task_result_callback(TaskResultCallback cb) { task_result_cb_ = cb; }
  void set_pipeline_request_callback(PipelineRequestCallback cb) { pipeline_request_cb_ = cb; }
  void set_create_image_sub_callback(CreateImageSubCallback cb) { create_image_sub_cb_ = cb; }

private:
  QRCodeResultCallback qrcode_result_cb_;
  QRCodeFeedbackCallback qrcode_feedback_cb_;
  TaskResultCallback task_result_cb_;
  PipelineRequestCallback pipeline_request_cb_;
  CreateImageSubCallback create_image_sub_cb_;
};

}  // namespace qrb::vision_manager
#endif