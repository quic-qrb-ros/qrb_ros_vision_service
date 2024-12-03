// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#include "qrb_vision_manager/camera_manager.hpp"

namespace qrb::vision_manager
{
static std::shared_ptr<CameraManager> singleton = nullptr;

std::shared_ptr<CameraManager> CameraManager::get_instance()
{
  if (singleton == nullptr) {
    singleton = std::shared_ptr<CameraManager>(new CameraManager());
  }
  return singleton;
}

CameraManager::CameraManager() {}

bool CameraManager::camera_available(const uint8_t camera_id)
{
  std::unique_lock<std::mutex> lock(mtx_);
  bool ret = true;
  for (auto status : cam_status_list_) {
    if (camera_id == status.camera_id) {
      ret = false;
      break;
    }
  }
  return ret;
}

bool CameraManager::can_stop_camera(const uint8_t camera_id)
{
  std::unique_lock<std::mutex> lock(mtx_);
  bool ret = false;
  for (auto status : cam_status_list_) {
    if (camera_id == status.camera_id && status.call_number == 1) {
      ret = true;
      break;
    }
  }
  return ret;
}

void CameraManager::notify_camera_start(const CameraStatus & camera)
{
  std::unique_lock<std::mutex> lock(mtx_);
  // search for the camera status
  for (auto status : cam_status_list_) {
    if (camera.camera_id == status.camera_id) {
      status.call_number++;
      return;
    }
  }
  // can not search for the camera
  cam_status_list_.emplace_back(camera);
}

bool CameraManager::get_camera_status(const uint8_t camera_id, CameraStatus & status)
{
  for (auto it : cam_status_list_) {
    if (camera_id == it.camera_id) {
      status = it;
      return true;
    }
  }
  return false;
}

void CameraManager::notify_camera_stop(const uint8_t camera_id)
{
  std::unique_lock<std::mutex> lock(mtx_);
  for (auto it = cam_status_list_.begin(); it != cam_status_list_.end(); it++) {
    if (camera_id == (*it).camera_id) {
      // remove the camera
      if ((*it).call_number == 1) {
        cam_status_list_.erase(it);
        break;
      }
      // not remove the camera
      (*it).call_number--;
      break;
    }
  }
}

}  // namespace qrb::vision_manager