// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#include "qrb_vision_manager/executor/executor_base.hpp"

#include <iostream>

using namespace std;

namespace qrb::vision_manager
{
const std::string ExecutorBase::CAMERA = "camera";
const std::string ExecutorBase::VIDEO = "video";
const std::string ExecutorBase::OBJECT_DETECT = "object_detect";
const std::string ExecutorBase::IMAGE_SEGEMENT = "image_segement";
const std::string ExecutorBase::QRCODE_DETECT = "qrcode_detect";

ExecutorBase::ExecutorBase(std::shared_ptr<CallbackManager> & cb_manager, const uint8_t camera_id)
  : cb_manager_(cb_manager), camera_id_(camera_id)
{
  parameter_manager_ = ParameterManager::get_instance();
}

bool ExecutorBase::get_parameter_from_node_info(const std::vector<std::string> & param_name_list,
    const std::string & parameter_name,
    int & index)
{
  auto it = std::find(param_name_list.begin(), param_name_list.end(), parameter_name);
  if (it == param_name_list.end()) {
    return false;
  }
  index = std::distance(param_name_list.begin(), it);
  return true;
}

bool ExecutorBase::get_camera_node_info(ComponentInfo & component_info,
    bool & need_resize,
    int & camera_height,
    int & camera_width,
    int require_height,
    int require_width)
{
  NodeInfo info;
  std::string node_name = "camera";
  need_resize = false;
  bool ret = parameter_manager_->get_node_parameter(node_name, info);
  if (!ret) {
    std::cout << "[ExecutorBase]: can not find this node parameter: " << node_name << std::endl;
    return false;
  }
  component_info.node_name = node_name + std::to_string(camera_id_);
  component_info.package_name = info.package_name;
  component_info.plugin_name = info.plugin_name;
  component_info.pub_topic_name = info.pub_topic_name;
  component_info.sub_topic_name = info.sub_topic_name;

  int index;
  std::string camera_id = "cameraId,int," + std::to_string(camera_id_);
  component_info.parameters.push_back(camera_id);

  auto param_list = info.parameter_list.find(std::to_string(camera_id_))->second;

  ret = add_parameter_to_component_info(
      info.parameter_name_list, param_list, "camera_info_path", component_info.parameters);
  if (!ret) {
    return false;
  }

  ret = get_parameter_from_node_info(info.parameter_name_list, "height", index);
  if (!ret) {
    std::cout << "[ExecutorBase]: can not find height parameter" << std::endl;
    return false;
  }
  std::string height_param = param_list.at(index)[0] + "," + param_list.at(index)[1] + ",";
  std::vector<std::string> support_height = param_list.at(index);
  ret = get_parameter_from_node_info(info.parameter_name_list, "width", index);
  if (!ret) {
    std::cout << "[ExecutorBase]: can not find width parameter" << std::endl;
    return false;
  }
  std::string width_param = param_list.at(index)[0] + "," + param_list.at(index)[1] + ",";
  std::vector<std::string> support_width = param_list.at(index);

  if (support_height.size() <= 2 || support_width.size() <= 2 ||
      support_height.size() != support_width.size()) {
    return false;
  }

  bool height_order = std::stoi(support_height[2]) > std::stoi(*(support_height.end() - 1));
  bool width_order = std::stoi(support_width[2]) > std::stoi(*(support_width.end() - 1));

  if (height_order != width_order) {
    return false;
  }
  if (require_height == 0 && require_width == 0) {
    height_param += height_order ? *(support_height.end() - 1) : support_height[2];
    camera_height =
        height_order ? std::stoi(*(support_height.end() - 1)) : std::stoi(support_height[2]);
    width_param += width_order ? *(support_width.end() - 1) : support_width[2];
    camera_width =
        width_order ? std::stoi(*(support_width.end() - 1)) : std::stoi(support_width[2]);
    component_info.parameters.push_back(height_param);
    component_info.parameters.push_back(width_param);
    return true;
  }

  int i = height_order ? support_height.size() - 1 : 2;
  for (;;) {
    camera_height = std::stoi(support_height[i]);
    camera_width = std::stoi(support_width[i]);
    if (camera_height >= require_height && camera_width >= require_width) {
      height_param += support_height[i];
      width_param += support_width[i];
      need_resize = camera_height != require_height || camera_width != require_width;
      break;
    }
    if (height_order) {
      if (i == 2) {
        height_param += support_height[i];
        width_param += support_width[i];
        need_resize = true;
        return false;
      }
      i--;
    } else {
      if (i == support_height.size() - 1) {
        height_param += support_height[i];
        width_param += support_width[i];
        need_resize = true;
        return false;
      }
      i++;
    }
  }
  component_info.parameters.push_back(height_param);
  component_info.parameters.push_back(width_param);
  return true;
}

bool ExecutorBase::get_resize_node_info(ComponentInfo & component_info,
    int require_height,
    int require_width)
{
  NodeInfo info;
  bool ret = parameter_manager_->get_node_parameter("resize", info);
  if (!ret) {
    std::cout << "[ExecutorBase]: can not find this node parameter: resize" << std::endl;
    return false;
  }
  component_info.node_name = "resize";
  component_info.package_name = info.package_name;
  component_info.plugin_name = info.plugin_name;
  component_info.pub_topic_name = info.pub_topic_name;
  component_info.sub_topic_name = info.sub_topic_name;

  auto param_list = info.parameter_list.begin()->second;

  std::string width_param;
  ret = get_parameter_to_component_info(info.parameter_name_list, param_list, "width", width_param);
  width_param += std::to_string(require_width);
  component_info.parameters.push_back(width_param);

  std::string height_param;
  ret =
      get_parameter_to_component_info(info.parameter_name_list, param_list, "height", height_param);
  height_param += std::to_string(require_height);
  component_info.parameters.push_back(height_param);

  return true;
}

bool ExecutorBase::get_color_convert_node_info(ComponentInfo & component_info, int require_type)
{
  NodeInfo info;
  bool ret = parameter_manager_->get_node_parameter("color_convert", info);
  if (!ret || require_type >= 2) {
    std::cout << "[ExecutorBase]: can not find this node parameter: color-convert" << std::endl;
    return false;
  }
  component_info.node_name = "color_convert";
  component_info.package_name = info.package_name;
  component_info.plugin_name = info.plugin_name;
  component_info.pub_topic_name = info.pub_topic_name;
  component_info.sub_topic_name = info.sub_topic_name;

  auto param_list = info.parameter_list.begin()->second;
  ret = add_parameter_to_component_info(info.parameter_name_list, param_list, "conversion_type",
      component_info.parameters, require_type);

  return ret;
}

bool ExecutorBase::get_nn_node_info(ComponentInfo & component_info, int require_type)
{
  NodeInfo info;
  bool ret = parameter_manager_->get_node_parameter("nn", info);
  if (!ret) {
    std::cout << "[ExecutorBase]: can not find this node parameter: nn" << std::endl;
    return false;
  }
  component_info.node_name = "nn_node";
  component_info.package_name = info.package_name;
  component_info.plugin_name = info.plugin_name;
  component_info.pub_topic_name = info.pub_topic_name;
  component_info.sub_topic_name = info.sub_topic_name;

  std::string type;

  switch (require_type) {
    case 0:
      type = "object_detect";
      break;
    case 1:
      type = "image_segment";
      break;
    default:
      break;
  }

  auto param_list = (info.parameter_list.find(type))->second;

  ret = add_parameter_to_component_info(
      info.parameter_name_list, param_list, "model_path", component_info.parameters);
  return ret;
}

bool ExecutorBase::get_yolo_object_det_post_process_node_info(ComponentInfo & component_info)
{
  NodeInfo info;
  bool ret = parameter_manager_->get_node_parameter("yolo_object_det_post_process", info);
  if (!ret) {
    std::cout << "[ExecutorBase]: can not find this node parameter: yolo_object_det_post_process"
              << std::endl;
    return false;
  }
  component_info.node_name = "yolo_object_det_post_process";
  component_info.package_name = info.package_name;
  component_info.plugin_name = info.plugin_name;
  component_info.pub_topic_name = info.pub_topic_name;
  component_info.sub_topic_name = info.sub_topic_name;

  auto param_list = info.parameter_list.begin()->second;

  ret = add_parameter_to_component_info(
      info.parameter_name_list, param_list, "label_file", component_info.parameters);
  if (!ret) {
    return false;
  }

  ret = add_parameter_to_component_info(
      info.parameter_name_list, param_list, "score_thres", component_info.parameters);
  if (!ret) {
    return false;
  }

  ret = add_parameter_to_component_info(
      info.parameter_name_list, param_list, "iou_thres", component_info.parameters);
  return ret;
}

bool ExecutorBase::get_yolo_pre_process_node_info(ComponentInfo & component_info)
{
  NodeInfo info;
  bool ret = parameter_manager_->get_node_parameter("yolo_pre_process", info);
  if (!ret) {
    std::cout << "[ExecutorBase]: can not find this node parameter: yolo_pre_process" << std::endl;
    return false;
  }
  component_info.node_name = "yolo_pre_process_node";
  component_info.package_name = info.package_name;
  component_info.plugin_name = info.plugin_name;
  component_info.pub_topic_name = info.pub_topic_name;
  component_info.sub_topic_name = info.sub_topic_name;

  return true;
}

bool ExecutorBase::get_yolo_image_seg_post_process_node_info(ComponentInfo & component_info)
{
  NodeInfo info;
  bool ret = parameter_manager_->get_node_parameter("yolo_image_seg_post_process", info);
  if (!ret) {
    std::cout << "[ExecutorBase]: can not find this node parameter: yolo_image_seg_post_process"
              << std::endl;
    return false;
  }
  component_info.node_name = "yolo_image_seg_post_process";
  component_info.package_name = info.package_name;
  component_info.plugin_name = info.plugin_name;
  component_info.pub_topic_name = info.pub_topic_name;
  component_info.sub_topic_name = info.sub_topic_name;

  auto param_list = info.parameter_list.begin()->second;

  ret = add_parameter_to_component_info(
      info.parameter_name_list, param_list, "label_file", component_info.parameters);
  if (!ret) {
    return false;
  }

  ret = add_parameter_to_component_info(
      info.parameter_name_list, param_list, "score_thres", component_info.parameters);
  if (!ret) {
    return false;
  }

  ret = add_parameter_to_component_info(
      info.parameter_name_list, param_list, "iou_thres", component_info.parameters);
  return ret;
}

bool ExecutorBase::add_parameter_to_component_info(const std::vector<std::string> & param_name_list,
    const std::vector<std::vector<std::string>> & param_list,
    const std::string & param_name,
    std::vector<std::string> & parameters,
    int reindex)
{
  int index;
  bool ret = get_parameter_from_node_info(param_name_list, param_name, index);
  if (!ret) {
    std::cout << "[ExecutorBase]: can not find " << param_name << std::endl;
    return false;
  }
  std::string param = param_list.at(index)[0] + "," + param_list.at(index)[1] + "," +
                      param_list.at(index)[2 + reindex];
  parameters.push_back(param);
  return ret;
}

bool ExecutorBase::get_parameter_to_component_info(const std::vector<std::string> & param_name_list,
    const std::vector<std::vector<std::string>> & param_list,
    const std::string & param_name,
    std::string & param_output)
{
  int index;
  bool ret = get_parameter_from_node_info(param_name_list, param_name, index);
  if (!ret) {
    std::cout << "[ExecutorBase]: can not find " << param_name << std::endl;
    return false;
  }
  param_output = param_list.at(index)[0] + "," + param_list.at(index)[1] + ",";
  return ret;
}

bool ExecutorBase::get_video_node_info(ComponentInfo & component_info,
    int require_height,
    int require_width)
{
  NodeInfo info;
  bool ret = parameter_manager_->get_node_parameter("video", info);
  if (!ret) {
    std::cout << "[ExecutorBase]: can not find this node parameter: video" << std::endl;
    return false;
  }
  component_info.node_name = "video";
  component_info.package_name = info.package_name;
  component_info.plugin_name = info.plugin_name;
  component_info.pub_topic_name = info.pub_topic_name;
  component_info.sub_topic_name = info.sub_topic_name;

  auto param_list = info.parameter_list.begin()->second;
  std::string width_param;
  ret = get_parameter_to_component_info(info.parameter_name_list, param_list, "width", width_param);
  width_param += std::to_string(require_width);
  component_info.parameters.push_back(width_param);

  std::string height_param;
  ret =
      get_parameter_to_component_info(info.parameter_name_list, param_list, "height", height_param);
  height_param += std::to_string(require_height);
  component_info.parameters.push_back(height_param);

  return true;
}

}  // namespace qrb::vision_manager
