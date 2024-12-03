/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "qrb_ros_vision_service/vision_service.hpp"

#include <chrono>

using namespace qrb::vision_manager;

namespace qrb_ros::vision_service
{
VisionService::VisionService()
{
  using namespace std::placeholders;
  node_ = std::make_shared<rclcpp::Node>("vision_service");
  request_manager_ = std::make_shared<RequestManager>();
  action_handler_ = std::make_shared<ActionHandler>(node_, request_manager_);
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  pipeline_client_ = node_->create_client<Pipeline>("/pipeline_manager_service");
  vision_task_srv_ = node_->create_service<VisionTask>("vision_task",
      std::bind(&VisionService::handle_task_service, this, _1, _2, _3),
      rmw_qos_profile_services_default, callback_group_);
  response_ = std::make_shared<VisionTask::Response>();
  request_manager_->register_pipeline_request_callback(
      std::bind(&VisionService::send_pipeline_request, this, _1, _2, _3, _4, _5, _6));
  request_manager_->register_task_result_callback(
      std::bind(&VisionService::send_task_result, this, _1, _2, _3, _4));
}

void VisionService::run()
{
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_);
  executor.spin();
}

void VisionService::handle_task_service(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<VisionTask::Request> request,
    const std::shared_ptr<VisionTask::Response> response)
{
  std::unique_lock<std::mutex> lock(mtx_);
  if (request_executed_) {
    response->success = false;
    response->err_info = "Please send request again later for there are executing one request";
    return;
  }
  auto fun = [this, &request]() {
    request_manager_->execute_task(
        request->task_id, request->camera_id, request->height, request->width);
  };
  std::thread{ fun }.detach();
  request_executed_ = true;
  cv.wait(lock);
  request_executed_ = false;
  response->success = response_->success;
  response->node_name = response_->node_name;
  response->output_topic_name = response_->output_topic_name;
  response->err_info = response_->err_info;
}

void VisionService::parse_parameter(const std::vector<std::string> & info_parameters,
    std::vector<rcl_interfaces::msg::Parameter> & rcl_parameters)
{
  using namespace rcl_interfaces::msg;
  for (auto it = info_parameters.begin(); it != info_parameters.end(); it++) {
    int name_index = (*it).find(',');
    int value_index = (*it).rfind(',');
    std::string name = (*it).substr(0, name_index);
    std::string value_type = (*it).substr(name_index + 1, value_index - name_index - 1);
    std::string value = (*it).substr(value_index + 1);
    Parameter param;
    param.name = name;
    RCLCPP_DEBUG(node_->get_logger(), "name: %s", name.c_str());
    RCLCPP_DEBUG(node_->get_logger(), "name: %s", value.c_str());
    if (value_type == "int") {
      param.value.type = rclcpp::ParameterType::PARAMETER_INTEGER;
      param.value.integer_value = std::stoi(value);
      RCLCPP_DEBUG(node_->get_logger(), "value: %d", param.value.integer_value);
    } else if (value_type == "bool") {
      param.value.type = rclcpp::ParameterType::PARAMETER_BOOL;
      if (value == "False") {
        param.value.bool_value = false;
      } else if (value == "True") {
        param.value.bool_value = true;
      }
    } else if (value_type == "double") {
      param.value.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
      param.value.double_value = std::stod(value);
      RCLCPP_DEBUG(node_->get_logger(), "value: %f", param.value.double_value);
    } else if (value_type == "string") {
      param.value.type = rclcpp::ParameterType::PARAMETER_STRING;
      param.value.string_value = value;
      RCLCPP_DEBUG(node_->get_logger(), "value: %s", value.c_str());
    }
    rcl_parameters.push_back(param);
  }
}

bool VisionService::send_pipeline_request(int action,
    int pipeline_id,
    const std::vector<ComponentInfo> & infos,
    const std::vector<int> & ids,
    std::string & output_topic,
    int & pipeline)
{
  RCLCPP_INFO(node_->get_logger(), "send pipeline request call");
  if (!pipeline_client_->wait_for_service(std::chrono::seconds(2))) {
    response_->success = false;
    response_->err_info = "Pipeline Manager service not available";
    return false;
  }
  auto request = std::make_shared<Pipeline::Request>();
  request->action = action;
  request->pipeline_id = pipeline_id;
  if (action != 0 && action != 1) {
    // load or unload node
    qrb_ros_pipeline_manager_msgs::msg::NodeInfo info;
    for (auto it = infos.begin(); it != infos.end(); it++) {
      RCLCPP_INFO(node_->get_logger(), "name: %s", (*it).node_name.c_str());
      info.node_name = (*it).node_name;
      info.package_name = (*it).package_name;
      info.plugin_name = (*it).plugin_name;
      if ((*it).pub_topic_name.size() != 0) {
        info.pub_topic_name = (*it).pub_topic_name[0];
        RCLCPP_DEBUG(node_->get_logger(), "pub_topic_name: %s", info.pub_topic_name.c_str());
      }
      if ((*it).sub_topic_name.size() != 0) {
        info.sub_topic_name = (*it).sub_topic_name[0];
        RCLCPP_DEBUG(node_->get_logger(), "sub_topic_name: %s", info.sub_topic_name.c_str());
      }
      parse_parameter((*it).parameters, info.parameters);
      request->nodes_info.push_back(info);
    }
  }
  RCLCPP_INFO(node_->get_logger(), "send pipeline request");
  auto future = pipeline_client_->async_send_request(request);
  auto future_wait = future.wait_for(std::chrono::seconds(10));
  if (future_wait != std::future_status::ready) {
    RCLCPP_INFO(node_->get_logger(), "future status is timeout");
    response_->success = false;
    response_->err_info = "Pipeline Manager service execute time out";
    return false;
  }
  auto result = future.get();
  if (result->success) {
    output_topic = result->output_topic_name;
    pipeline = result->pipeline_id;
  }
  return result->success;
}

void VisionService::send_task_result(const std::vector<std::string> & topic_name,
    const std::vector<std::string> & node_name,
    const std::string & error_info,
    bool success)
{
  RCLCPP_INFO(node_->get_logger(), "send task result call");
  response_->success = success;
  response_->node_name.resize(node_name.size());
  std::copy(node_name.begin(), node_name.end(), response_->node_name.begin());
  response_->output_topic_name.resize(topic_name.size());
  std::copy(topic_name.begin(), topic_name.end(), response_->output_topic_name.begin());
  response_->err_info = error_info;
  cv.notify_one();
  return;
}

}  // namespace qrb_ros::vision_service
