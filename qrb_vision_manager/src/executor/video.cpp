// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#include "qrb_vision_manager/executor/video.hpp"

#include <iostream>

namespace qrb::vision_manager
{
VideoExecutor::VideoExecutor(std::shared_ptr<CallbackManager> & cb_manager, const uint8_t camera_id)
  : ExecutorBase(cb_manager, camera_id)
{
}

VideoExecutor::~VideoExecutor()
{
  status_list_.clear();
}

bool VideoExecutor::enable(int height, int width, CameraStatus & cam_status)
{
  std::vector<ComponentInfo> component_infos;

  bool need_resize = false;
  ComponentInfo camera_info;
  int camera_height, camera_width;

  bool ret =
      get_camera_node_info(camera_info, need_resize, camera_height, camera_width, height, width);
  if (!ret) {
    std::cout << "[VideoExecutor]: please add this camera name in config file: camera_"
              << std::to_string(camera_id_) << std::endl;
    cb_manager_->send_task_result({}, {}, "please add camera config in config file", false);
    return false;
  }
  component_infos.push_back(camera_info);

  ComponentInfo resize_info;
  if (need_resize) {
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

  ComponentInfo video_info;
  ret = get_video_node_info(video_info, height, width);
  if (!ret) {
    cb_manager_->send_task_result({}, {},
        "please add video_info config in config file due to can not meet "
        "requirement",
        false);
    return false;
  }
  component_infos.push_back(video_info);

  std::string topic_name;
  int pipeline;
  ret = cb_manager_->send_pipeline_request(TaskCommand::PIPELINE_CREATE, pipeline_id_,
      component_infos, std::vector<int>(), topic_name, pipeline);
  std::cout << "[VideoExecutor]: send create pipeline request" << std::endl;
  if (!ret) {
    cb_manager_->send_task_result({}, {}, "create pipeline failed", false);
    return false;
  }
  pipeline_id_ = pipeline;
  ret = cb_manager_->send_pipeline_request(TaskCommand::PIPELINE_LOAD, pipeline_id_,
      component_infos, std::vector<int>(), topic_name, pipeline);
  std::cout << "[VideoExecutor]: send pipeline load request" << std::endl;
  if (!ret) {
    cb_manager_->send_task_result({}, {}, "send pipeline request failed", false);
    return false;
  }

  int index = topic_name.rfind('/');
  std::string prefix_name = topic_name.substr(0, index + 1);
  ComponentStatus status;
  status.topic_name = need_resize ? prefix_name + camera_info.pub_topic_name[0] : topic_name;
  status.node_name = "camera";
  status.call_number = 1;
  status.height = camera_height;
  status.width = camera_width;
  status.pipeline_id = pipeline_id_;
  std::cout << "[VideoExecutor]: camera topic name " << status.topic_name << std::endl;

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
    status.topic_name = prefix_name + resize_info.pub_topic_name[0];
    status.node_name = "resize";
    status.call_number = 1;
    status.height = height;
    status.width = width;
    status.pipeline_id = pipeline_id_;
    status_list_.push_back(status);
    std::cout << "[VideoExecutor]: resize topic name " << status.topic_name << std::endl;
  }

  status.topic_name = topic_name;
  status.node_name = "video";
  status.call_number = 1;
  status.height = height;
  status.width = width;
  status.pipeline_id = pipeline_id_;
  status_list_.push_back(status);
  std::cout << "[VideoExecutor]: video topic name " << status.topic_name << std::endl;

  node_name.push_back("video");
  topic_list.push_back(topic_name);
  cb_manager_->send_task_result(topic_list, node_name, "", true);
  listener_ = 1;
  return true;
}

bool VideoExecutor::stop(int height, int width, bool stop_camera)
{
  if (status_list_.size() == 0) {
    cb_manager_->send_task_result({}, {}, "please add this task first", false);
    std::cout << "[VideoExecutor]: please call enable first." << std::endl;
    return false;
  }
  if (listener_ > 1) {
    cb_manager_->send_task_result({}, {}, "", true);
    std::cout << "[VideoExecutor]: unregister success." << std::endl;
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

void VideoExecutor::add_new_listener(int height, int width)
{
  // todo: future support upscale, currently only support output the same resolution one.
  std::cout << "[VideoExecutor]: new resolution request." << std::endl;
  std::vector<std::string> node_name;
  std::vector<std::string> topic_name;

  for (auto status : status_list_) {
    // todo: add "image" source in the future
    if (status.node_name != "video") {
      continue;
    }
    node_name.push_back(status.node_name);
    topic_name.push_back(status.topic_name);
  }

  if (node_name.empty()) {
    cb_manager_->send_task_result({}, {},
        "There is no node here to meet this requirement, please stop "
        "this camera and start the new one",
        false);
    return;
  }

  cb_manager_->send_task_result(topic_name, node_name, "", true);
  listener_++;
}

}  // namespace qrb::vision_manager