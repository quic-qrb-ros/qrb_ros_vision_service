// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#include "qrb_vision_manager/executor/camera.hpp"

#include <iostream>

namespace qrb::vision_manager
{
CameraExecutor::CameraExecutor(std::shared_ptr<CallbackManager> & cb_manager,
    const uint8_t camera_id)
  : ExecutorBase(cb_manager, camera_id)
{
}

CameraExecutor::~CameraExecutor()
{
  status_list_.clear();
}

bool CameraExecutor::enable(int height, int width, CameraStatus & cam_status)
{
  std::vector<ComponentInfo> component_infos;

  bool need_resize = false;
  ComponentInfo camera_info;
  int camera_height, camera_width;

  bool ret =
      get_camera_node_info(camera_info, need_resize, camera_height, camera_width, height, width);
  if (!ret) {
    std::cout << "[CameraExecutor]: please add this camera name in config file: camera_"
              << std::to_string(camera_id_) << std::endl;
    cb_manager_->send_task_result({}, {}, "please add camera config in config file", false);
    return false;
  }
  component_infos.push_back(camera_info);

  if (need_resize) {
    ComponentInfo resize_info;
    ret = get_resize_node_info(resize_info, height, width);
    if (!ret) {
      cb_manager_->send_task_result({}, {},
          "please add resize config in config file due to camera "
          "resolution can not meet requirement",
          false);
      return false;
    }
    component_infos.push_back(resize_info);
  }

  std::string topic_name;
  int pipeline;
  ret = cb_manager_->send_pipeline_request(TaskCommand::PIPELINE_CREATE, pipeline_id_,
      component_infos, std::vector<int>(), topic_name, pipeline);
  std::cout << "[CameraExecutor]: send create pipeline request" << std::endl;
  if (!ret) {
    cb_manager_->send_task_result({}, {}, "create pipeline failed", false);
    return false;
  }
  pipeline_id_ = pipeline;
  ret = cb_manager_->send_pipeline_request(TaskCommand::PIPELINE_LOAD, pipeline_id_,
      component_infos, std::vector<int>(), topic_name, pipeline);
  std::cout << "[CameraExecutor]: send pipeline load request" << std::endl;
  if (!ret) {
    cb_manager_->send_task_result({}, {}, "send pipeline request failed", false);
    return false;
  }

  int index = topic_name.rfind('/');
  std::string prefix_name = topic_name.substr(0, index + 1);
  ComponentStatus status;
  status.topic_name = need_resize ? prefix_name + camera_info.pub_topic_name[0] : topic_name;
  status.call_number = 1;
  status.height = camera_height;
  status.width = camera_width;
  status.pipeline_id = pipeline_id_;
  std::cout << "[CameraExecutor]: camera topic name " << status.topic_name << std::endl;

  std::vector<std::string> node_name;
  std::vector<std::string> topic_list;

  cam_status.camera_id = camera_id_;
  cam_status.topic_name = need_resize ? prefix_name + camera_info.pub_topic_name[0] : topic_name;
  cam_status.call_number = 1;
  cam_status.height = camera_height;
  cam_status.width = camera_width;
  cam_status.pipeline_id = pipeline_id_;

  status_list_.push_back(status);
  // storage resize info when need resize node
  if (need_resize) {
    status.topic_name = topic_name;
    status.call_number = 1;
    status.height = height;
    status.width = width;
    status.pipeline_id = pipeline_id_;
    status_list_.push_back(status);
  }
  node_name.push_back("camera");
  topic_list.push_back(topic_name);
  cb_manager_->send_task_result(topic_list, node_name, "", true);
  listener_ = 1;
  return true;
}

bool CameraExecutor::stop(int height, int width, bool stop_camera)
{
  if (status_list_.size() == 0) {
    cb_manager_->send_task_result({}, {}, "please add this task first", false);
    std::cout << "[CameraExecutor]: please call enable first." << std::endl;
    return false;
  }
  if (listener_ > 1) {
    cb_manager_->send_task_result({}, {}, "", true);
    std::cout << "[CameraExecutor]: unregister success." << std::endl;
    listener_--;
    return false;
  }
  std::string topic_name;
  int pipeline;
  std::vector<ComponentInfo> info;
  std::vector<int> ids;
  // cancel all the node
  if (height == 0 && width == 0) {
    bool ret;
    if (stop_camera) {
      ret = cb_manager_->send_pipeline_request(
          TaskCommand::PIPELINE_DESTROY, pipeline_id_, info, ids, topic_name, pipeline);
    } else {
      // remove this task 's node in pipeline
      for (auto it = status_list_.begin() + 1; it != status_list_.end(); it++) {
        ids.push_back((*it).unique_id);
      }
      ret = cb_manager_->send_pipeline_request(
          TaskCommand::PIPELINE_UNLOAD, pipeline_id_, info, ids, topic_name, pipeline);
    }
    if (ret) {
      status_list_.clear();

      cb_manager_->send_task_result({}, {}, "", true);
    } else {
      cb_manager_->send_task_result({}, {}, "task execute failed for pipeline service", false);
    }
    return ret;
  }

  for (auto it = status_list_.begin(); it != status_list_.end(); it++) {
    if ((*it).height == height && (*it).width == width) {
      if ((*it).call_number != 1) {
        cb_manager_->send_task_result({}, {}, "can not stop this task due to other use", false);
        (*it).call_number--;
        if (it != status_list_.begin()) {
          status_list_[0].call_number--;
        }
        return false;
      }
      bool ret;
      // 能够单独停止这个node
      if (status_list_[0].call_number == 1 && stop_camera) {
        ret = cb_manager_->send_pipeline_request(
            TaskCommand::PIPELINE_DESTROY, pipeline_id_, info, ids, topic_name, pipeline);
      } else {
        ids.push_back((*it).unique_id);
        ret = cb_manager_->send_pipeline_request(
            TaskCommand::PIPELINE_UNLOAD, pipeline_id_, info, ids, topic_name, pipeline);
      }
      if (ret) {
        cb_manager_->send_task_result({}, {}, "", true);
        status_list_.erase(it);
        if (status_list_[0].call_number == 1) {
          return true;
        }
        return false;
      }
      cb_manager_->send_task_result({}, {}, "task execute failed for pipeline service", false);
      return false;
    }
  }
  cb_manager_->send_task_result({}, {}, "this task did not start", false);
  return false;
}

const std::string CameraExecutor::get_topic_name(int height, int width)
{
  for (auto it = status_list_.begin(); it != status_list_.end(); it++) {
    if ((*it).height == height && (*it).width == width) {
      (*it).call_number++;
      if (it != status_list_.begin()) {
        status_list_[0].call_number++;
      }
      return (*it).topic_name;
    }
  }
  return "";
}

void CameraExecutor::add_new_listener(int height, int width)
{
  // todo: future support upscale, currently only support output the same resolution one.
  std::cout << "[CameraExecutor]: add new listener." << std::endl;
  int camera_height = status_list_[0].height;
  int camera_width = status_list_[0].width;
  if (height > camera_height && width > camera_width) {
    cb_manager_->send_task_result(
        {}, {}, "can not exeucte this task for resolution is higher than camera", false);
    return;
  }

  const std::string meet_require_topic = get_topic_name(height, width);

  if (meet_require_topic != "") {
    std::vector<std::string> node_name;
    std::vector<std::string> topic_name;
    node_name.push_back("camera");
    topic_name.push_back(meet_require_topic);
    cb_manager_->send_task_result(topic_name, node_name, "", true);
    listener_++;
    return;
  }
  cb_manager_->send_task_result({}, {},
      "There is no node here to meet this requirement, please stop this "
      "camera and start the new one",
      false);
}

}  // namespace qrb::vision_manager