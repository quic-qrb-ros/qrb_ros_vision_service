// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#include "qrb_vision_manager/request_manager.hpp"

namespace qrb::vision_manager
{
RequestManager::RequestManager()
{
  cb_manager_ = std::make_shared<CallbackManager>();
  mission_manager_ = std::make_shared<MissionManager>(cb_manager_);
}

void RequestManager::register_qrcode_result_callback(QRCodeResultCallback cb)
{
  cb_manager_->set_qrcode_result_callback(cb);
}

void RequestManager::register_qrcode_feedback_callback(QRCodeFeedbackCallback cb)
{
  cb_manager_->set_qrcode_feedback_callback(cb);
}

void RequestManager::register_task_result_callback(TaskResultCallback cb)
{
  cb_manager_->set_task_result_callback(cb);
}

void RequestManager::register_pipeline_request_callback(PipelineRequestCallback cb)
{
  cb_manager_->set_pipeline_request_callback(cb);
}

void RequestManager::register_create_image_sub_callback(CreateImageSubCallback cb)
{
  cb_manager_->set_create_image_sub_callback(cb);
}

void RequestManager::execute_task(const int type,
    const uint8_t camera_id,
    const int height,
    const int width)
{
  switch (type) {
    case TaskCommand::REGISTER_CAMERA:
    case TaskCommand::REGISTER_VIDEO:
    case TaskCommand::REGISTER_OBJECT_DETECT:
    case TaskCommand::REGISTER_IMAGE_SEGEMENT:
      mission_manager_->start_task(type, camera_id, height, width);
      break;
    case TaskCommand::UNREGISTER_CAMERA:
    case TaskCommand::UNREGISTER_VIDEO:
    case TaskCommand::UNREGISTER_OBJECT_DETECT:
    case TaskCommand::UNREGISTER_IMAGE_SEGEMENT:
      mission_manager_->stop_task(type, camera_id, height, width);
      break;
    default:
      cb_manager_->send_task_result({}, {}, "do not support this command type", false);
      break;
  }
}

// process start file case
bool RequestManager::execute_qr_case(const int type, const std::vector<std::string> & names)
{
  bool parse_success = false;
  switch (type) {
    case TaskCommand::START_QR_CODE_CASE_FILES:
      mission_manager_->start_qr_case(type, names);
      parse_success = true;
      break;
    default:
      std::cout << "do not understand this qr case command: " << type << std::endl;
      break;
  }
  return parse_success;
}

// process stop case & start stream case
bool RequestManager::execute_qr_case(const int type, const uint8_t camera_id)
{
  bool parse_success = false;
  switch (type) {
    case TaskCommand::REGISTER_CAMERA_QR_DETECT:
      mission_manager_->start_qr_case(type, camera_id);
      parse_success = true;
      break;
    case TaskCommand::UNREGISTER_CAMERA_QR_DETECT:
    case TaskCommand::STOP_FILE_CASE:
      parse_success = mission_manager_->stop_qr_case(type, camera_id);
      break;
    default:
      std::cout << "do not understand this qr case command: " << type << std::endl;
      break;
  }
  return parse_success;
}

bool RequestManager::execute_qr_case(const int type, const std::string & name)
{
  bool parse_success = false;
  switch (type) {
    case TaskCommand::REGISTER_TOPIC_QR_DETECT:
      mission_manager_->start_qr_case(type, name);
      parse_success = true;
      break;
    case TaskCommand::UNREGISTER_TOPIC_QR_DETECT:
      mission_manager_->stop_qr_case(type, name);
      parse_success = true;
      break;
    default:
      std::cout << "do not understand this qr case command: " << type << std::endl;
      break;
  }
  return parse_success;
}

}  // namespace qrb::vision_manager