/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef QRB_ROS_VISION_SERVICE__VISION_SERVICE_HPP__
#define QRB_ROS_VISION_SERVICE__VISION_SERVICE_HPP__

#include "qrb_ros_pipeline_manager_msgs/srv/pipeline.hpp"
#include "qrb_ros_vision_service/action_handler.hpp"
#include "qrb_ros_vision_service_msgs/srv/vision_task.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qrb_ros::vision_service
{
class VisionService
{
  using VisionTask = qrb_ros_vision_service_msgs::srv::VisionTask;
  using Pipeline = qrb_ros_pipeline_manager_msgs::srv::Pipeline;

public:
  explicit VisionService();
  void run();

private:
  void handle_task_service(const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<VisionTask::Request> request,
      const std::shared_ptr<VisionTask::Response> response);

  void send_task_result(const std::vector<std::string> & topic_name,
      const std::vector<std::string> & node_name,
      const std::string & error_info,
      bool success);

  bool send_pipeline_request(int action,
      int pipeline_id,
      const std::vector<qrb::vision_manager::ComponentInfo> & infos,
      const std::vector<int> & ids,
      std::string & topic_name,
      int & pipeline);

  void parse_parameter(const std::vector<std::string> & info_parameters,
      std::vector<rcl_interfaces::msg::Parameter> & rcl_parameters);

  bool request_executed_{ false };

  std::shared_ptr<ActionHandler> action_handler_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Pipeline>::SharedPtr pipeline_client_;
  rclcpp::Service<VisionTask>::SharedPtr vision_task_srv_;

  std::shared_ptr<qrb::vision_manager::RequestManager> request_manager_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::shared_ptr<VisionTask::Response> response_;
  std::mutex mtx_;

  std::condition_variable cv;
};

}  // namespace qrb_ros::vision_service

#endif