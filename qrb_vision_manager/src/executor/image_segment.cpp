// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#include "qrb_vision_manager/executor/image_segment.hpp"

#include <iostream>

namespace qrb::vision_manager
{
ImageSegmentExecutor::ImageSegmentExecutor(std::shared_ptr<CallbackManager> & cb_manager,
    const uint8_t camera_id)
  : ExecutorBase(cb_manager, camera_id)
{
}

ImageSegmentExecutor::~ImageSegmentExecutor()
{
  status_list_.clear();
}

bool ImageSegmentExecutor::enable(int height, int width, CameraStatus & cam_status)
{
  std::vector<ComponentInfo> component_infos;

  ComponentInfo camera_info;
  int camera_height, camera_width;
  bool need_resize;
  bool ret = get_camera_node_info(camera_info, need_resize, camera_height, camera_width, 640, 640);
  if (!ret) {
    std::cout << "[ImageSegmentExecutor]: please add this camera name in config file: camera_"
              << std::to_string(camera_id_) << std::endl;
    cb_manager_->send_task_result({}, {}, "please add camera config in config file", false);
    return false;
  }
  component_infos.push_back(camera_info);

  ComponentInfo resize_info;
  ret = get_resize_node_info(resize_info, 640, 640);
  if (!ret) {
    cb_manager_->send_task_result({}, {},
        "please add resize config in config file due to camera "
        "resolution can not meet requirement",
        false);
    return false;
  }
  component_infos.push_back(resize_info);

  ComponentInfo color_convert_info;
  ret = get_color_convert_node_info(color_convert_info, 0);
  if (!ret) {
    cb_manager_->send_task_result({}, {},
        "please add color convert config in config file due to image 's "
        "format can not meet requirement",
        false);
    return false;
  }
  component_infos.push_back(color_convert_info);

  ComponentInfo yolo_preprocess_info;
  ret = get_yolo_pre_process_node_info(yolo_preprocess_info);
  if (!ret) {
    cb_manager_->send_task_result({}, {},
        "please add nn config in config file due to object detect need "
        "it",
        false);
    return false;
  }
  component_infos.push_back(yolo_preprocess_info);

  ComponentInfo nn_info;
  ret = get_nn_node_info(nn_info, 0);
  if (!ret) {
    cb_manager_->send_task_result({}, {},
        "please add nn config in config file due to object detect need "
        "it",
        false);
    return false;
  }
  component_infos.push_back(nn_info);

  ComponentInfo postprocess_info;
  ret = get_yolo_image_seg_post_process_node_info(postprocess_info);
  if (!ret) {
    cb_manager_->send_task_result({}, {},
        "please add nn config in config file due to object detect need "
        "it",
        false);
    return false;
  }
  component_infos.push_back(postprocess_info);

  std::string topic_name;
  int pipeline;
  ret = cb_manager_->send_pipeline_request(TaskCommand::PIPELINE_CREATE, pipeline_id_,
      component_infos, std::vector<int>(), topic_name, pipeline);
  std::cout << "[ImageSegmentExecutor]: send create pipeline request" << std::endl;
  if (!ret) {
    cb_manager_->send_task_result({}, {}, "create pipeline failed", false);
    return false;
  }
  pipeline_id_ = pipeline;
  ret = cb_manager_->send_pipeline_request(TaskCommand::PIPELINE_LOAD, pipeline_id_,
      component_infos, std::vector<int>(), topic_name, pipeline);
  std::cout << "[ImageSegmentExecutor]: send pipeline load request" << std::endl;
  if (!ret) {
    cb_manager_->send_task_result({}, {}, "send pipeline request failed", false);
    return false;
  }

  std::vector<std::string> node_name;
  std::vector<std::string> topic_list;

  int index = topic_name.rfind('/');
  std::string prefix_name = topic_name.substr(0, index + 1);
  ComponentStatus status;
  status.node_name = "camera";
  status.topic_name = prefix_name + camera_info.pub_topic_name[0];
  status.call_number = 1;
  status.height = camera_height;
  status.width = camera_width;
  status.pipeline_id = pipeline_id_;
  status_list_.push_back(status);
  std::cout << "[ImageSegmentExecutor]: camera topic name " << status.topic_name << std::endl;

  cam_status.camera_id = camera_id_;
  cam_status.topic_name = prefix_name + camera_info.pub_topic_name[0];
  cam_status.call_number = 1;
  cam_status.height = camera_height;
  cam_status.width = camera_width;
  cam_status.pipeline_id = pipeline_id_;

  status.node_name = "resize";
  status.topic_name = prefix_name + resize_info.pub_topic_name[0];
  status.call_number = 1;
  status.height = 640;
  status.width = 640;
  status.pipeline_id = pipeline_id_;
  status_list_.push_back(status);
  std::cout << "[ImageSegmentExecutor]: resize topic name " << status.topic_name << std::endl;

  status.node_name = "image";
  status.topic_name = prefix_name + color_convert_info.pub_topic_name[0];
  status.call_number = 1;
  status.height = 640;
  status.width = 640;
  status.pipeline_id = pipeline_id_;
  status_list_.push_back(status);
  std::cout << "[ImageSegmentExecutor]: color_convert topic name " << status.topic_name
            << std::endl;

  status.node_name = "segment_output";
  status.topic_name = topic_name;
  status.call_number = 1;
  status.height = 640;
  status.width = 640;
  status.pipeline_id = pipeline_id_;
  status_list_.push_back(status);
  std::cout << "[ImageSegmentExecutor]: postprocess topic name " << status.topic_name << std::endl;

  // todo: add "image" source in the future
  node_name.push_back("segment_output");
  topic_list.push_back(topic_name);
  cb_manager_->send_task_result(topic_list, node_name, "", true);
  listener_ = 1;
  return true;
}

bool ImageSegmentExecutor::stop(int height, int width, bool stop_camera)
{
  if (status_list_.size() == 0) {
    cb_manager_->send_task_result({}, {}, "please add this task first", false);
    std::cout << "[ImageSegmentExecutor]: please call enable first." << std::endl;
    return false;
  }
  if (listener_ > 1) {
    cb_manager_->send_task_result({}, {}, "", true);
    std::cout << "[ImageSegmentExecutor]: unregister success." << std::endl;
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

  return false;
}

void ImageSegmentExecutor::add_new_listener(int height, int width)
{
  std::cout << "[ImageSegmentExecutor]: add new listener." << std::endl;

  std::vector<std::string> node_name;
  std::vector<std::string> topic_name;

  for (auto status : status_list_) {
    // todo: add "image" source in the future
    if (status.node_name != "segment_output") {
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