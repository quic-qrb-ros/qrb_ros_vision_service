// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#include "qrb_vision_manager/mission_manager.hpp"

#include <iostream>

#include "qrb_vision_manager/executor/camera.hpp"
#include "qrb_vision_manager/executor/image_segment.hpp"
#include "qrb_vision_manager/executor/object_detect.hpp"
#include "qrb_vision_manager/executor/qr_code_detect.hpp"
#include "qrb_vision_manager/executor/video.hpp"

namespace qrb::vision_manager
{
MissionManager::MissionManager(std::shared_ptr<CallbackManager> & cb_manager)
  : cb_manager_(cb_manager)
{
  param_manager_ = ParameterManager::get_instance();
  camera_manager_ = CameraManager::get_instance();
}

void MissionManager::start_task(const int type,
    const uint8_t camera_id,
    const int height,
    const int width)
{
  std::cout << "receive start task" << std::endl;

  std::string task_name = "";
  std::shared_ptr<ExecutorBase> base;
  switch (type) {
    case TaskCommand::REGISTER_CAMERA:
      task_name = ExecutorBase::CAMERA + "-" + std::to_string(camera_id);
      base = std::make_shared<CameraExecutor>(cb_manager_, camera_id);
      break;
    case TaskCommand::REGISTER_VIDEO:
      task_name = ExecutorBase::VIDEO + "-" + std::to_string(camera_id);
      base = std::make_shared<VideoExecutor>(cb_manager_, camera_id);
      break;
    case TaskCommand::REGISTER_OBJECT_DETECT:
      task_name = ExecutorBase::OBJECT_DETECT + "-" + std::to_string(camera_id);
      base = std::make_shared<ObjectDetectExecutor>(cb_manager_, camera_id);
      break;
    case TaskCommand::REGISTER_IMAGE_SEGEMENT:
      task_name = ExecutorBase::IMAGE_SEGEMENT + "-" + std::to_string(camera_id);
      base = std::make_shared<ImageSegmentExecutor>(cb_manager_, camera_id);
      break;
    default:
      break;
  }

  auto executor = check_executor_exist(task_name);
  if (executor != nullptr) {
    executor->add_new_listener(height, width);
    return;
  }

  bool camera_available = camera_manager_->camera_available(camera_id);
  if (!camera_available) {
    cb_manager_->send_task_result({}, {}, camera_id + " is not available", false);
    return;
  }

  CameraStatus cam_status;
  if (base->enable(height, width, cam_status)) {
    std::unique_lock<std::mutex> lock(mtx_);
    executor_maps_.insert(make_pair(task_name, base));
    camera_manager_->notify_camera_start(cam_status);
    return;
  }
}

void MissionManager::stop_task(const int type,
    const uint8_t camera_id,
    const int height,
    const int width)
{
  std::cout << "receive stop task" << std::endl;
  std::string task_name = "";
  switch (type) {
    case TaskCommand::UNREGISTER_CAMERA:
      task_name = ExecutorBase::CAMERA + "-" + std::to_string(camera_id);
      break;
    case TaskCommand::UNREGISTER_VIDEO:
      task_name = ExecutorBase::VIDEO + "-" + std::to_string(camera_id);
      break;
    case TaskCommand::UNREGISTER_OBJECT_DETECT:
      task_name = ExecutorBase::OBJECT_DETECT + "-" + std::to_string(camera_id);
      break;
    case TaskCommand::UNREGISTER_IMAGE_SEGEMENT:
      task_name = ExecutorBase::IMAGE_SEGEMENT + "-" + std::to_string(camera_id);
      break;
    default:
      break;
  }

  auto executor = check_executor_exist(task_name);
  if (executor == nullptr) {
    cb_manager_->send_task_result(
        {}, {}, "This task cannot be stopped because it has not yet started", false);
    return;
  }
  bool stop_camera = camera_manager_->can_stop_camera(camera_id);

  if (executor->stop(0, 0, stop_camera)) {
    remove_executor(task_name);
    camera_manager_->notify_camera_stop(camera_id);
  }
}

// file只支持同时处理一个
void MissionManager::start_qr_case(const int type, const std::vector<std::string> & names)
{
  std::string task_name = ExecutorBase::QRCODE_DETECT + "_files";

  if (check_executor_exist(task_name) != nullptr) {
    return;
  }
  std::string name = "file";
  auto base = std::make_shared<QRCodeDetectExecutor>(cb_manager_, name);
  bool ret = base->execute(names);
  executor_maps_.insert(make_pair(task_name, base));
}

void MissionManager::start_qr_case(const int type, const uint8_t camera_id)
{
  std::string task_name = ExecutorBase::QRCODE_DETECT + "-" + std::to_string(camera_id) + "_stream";

  if (check_executor_exist(task_name) != nullptr) {
    return;
  }

  bool camera_available = camera_manager_->camera_available(camera_id);
  if (!camera_available) {
    cb_manager_->send_qrcode_result(
        {}, camera_id + " is not available", false, "camera-" + std::to_string(camera_id));
    return;
  }

  auto base = std::make_shared<QRCodeDetectExecutor>(cb_manager_, camera_id);
  CameraStatus cam_status;
  bool ret = base->execute(camera_id, cam_status);
  if (ret) {
    std::unique_lock<std::mutex> lock(mtx_);
    executor_maps_.insert(make_pair(task_name, base));
    camera_manager_->notify_camera_start(cam_status);
  }
}

bool MissionManager::stop_qr_case(const int type, const uint8_t camera_id)
{
  std::cout << "receive remove qr case request" << std::endl;
  std::string task_name = "";
  switch (type) {
    case TaskCommand::UNREGISTER_CAMERA_QR_DETECT:
      task_name = ExecutorBase::QRCODE_DETECT + "-" + std::to_string(camera_id) + "_stream";
      break;
    case TaskCommand::STOP_FILE_CASE:
      task_name = ExecutorBase::QRCODE_DETECT + "_files";
      break;
    default:
      break;
  }

  auto executor = check_executor_exist(task_name);
  if (executor == nullptr) {
    return false;
  }

  bool stop_camera = camera_manager_->can_stop_camera(camera_id);

  bool ret = (std::dynamic_pointer_cast<QRCodeDetectExecutor>(executor))->stop_case(stop_camera);
  if (ret) {
    remove_executor(task_name);
    if (type == TaskCommand::UNREGISTER_CAMERA_QR_DETECT) {
      camera_manager_->notify_camera_stop(camera_id);
    }
  }
  return ret;
}

std::shared_ptr<ExecutorBase> MissionManager::check_executor_exist(const std::string & name)
{
  std::unique_lock<std::mutex> lock(mtx_);
  for (auto & it : executor_maps_) {
    if (it.first == name) {
      // executing this case
      return it.second;
    }
  }
  return nullptr;
}

void MissionManager::remove_executor(const std::string & name)
{
  std::unique_lock<std::mutex> lock(mtx_);
  executor_maps_.erase(name);
}

void MissionManager::start_qr_case(const int type, const std::string & name)
{
  std::string task_name = ExecutorBase::QRCODE_DETECT + "-" + name + "_topic";

  if (check_executor_exist(task_name) != nullptr) {
    return;
  }
  auto base = std::make_shared<QRCodeDetectExecutor>(cb_manager_, name);
  bool ret = base->execute();
  executor_maps_.insert(make_pair(task_name, base));
}

bool MissionManager::stop_qr_case(const int type, const std::string & name)
{
  std::cout << "receive remove qr case request" << std::endl;
  std::string task_name = ExecutorBase::QRCODE_DETECT + "-" + name + "_topic";

  auto executor = check_executor_exist(task_name);
  if (executor == nullptr) {
    return false;
  }

  bool ret = (std::dynamic_pointer_cast<QRCodeDetectExecutor>(executor))->stop_case();
  if (ret) {
    remove_executor(task_name);
  }
  return ret;
}

}  // namespace qrb::vision_manager