/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef QRB_ROS_VISION_SERVICE__ACTION_HANDLER_HPP__
#define QRB_ROS_VISION_SERVICE__ACTION_HANDLER_HPP__

#include <atomic>
#include <map>
#include <vector>

#include "opencv2/opencv.hpp"
#include "qrb_ros_vision_service_msgs/action/qr_code_case.hpp"
#include "qrb_vision_manager/request_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace qrb_ros::vision_service
{
class ActionHandler
{
  using QRCodeCase = qrb_ros_vision_service_msgs::action::QRCodeCase;
  using GoalHandleQRCodeCase = rclcpp_action::ServerGoalHandle<QRCodeCase>;
  using ImageCallback = std::function<void(cv::Mat & image)>;

public:
  explicit ActionHandler(std::shared_ptr<rclcpp::Node> node_handler,
      std::shared_ptr<qrb::vision_manager::RequestManager> require_manager);

  ~ActionHandler();

private:
  void qr_code_feedback(const std::vector<std::string> & content, const std::string & source);

  void qr_code_result(const std::vector<std::string> & content,
      const std::string & err_info,
      bool success,
      const std::string & source);

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const QRCodeCase::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleQRCodeCase> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleQRCodeCase> goal_handle);

  bool remove_goal_handle(std::vector<std::shared_ptr<GoalHandleQRCodeCase>> & goal_handle_list,
      const std::shared_ptr<GoalHandleQRCodeCase> goal_handle);

  void execute_stream_detect(const std::shared_ptr<GoalHandleQRCodeCase> goal_handle);
  void execute_stop_detect(const std::shared_ptr<GoalHandleQRCodeCase> goal_handle);
  void execute_file_detect(const std::shared_ptr<GoalHandleQRCodeCase> goal_handle);

  void create_image_sub(const std::string & source, const std::string & name, ImageCallback cb);

  bool is_case_working(const std::string & name);

  void remove_file_handler();

  std::mutex mtx_;

  std::map<std::string, std::vector<std::shared_ptr<GoalHandleQRCodeCase>>> goal_handle_map_;
  std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_sub_map_;
  std::shared_ptr<GoalHandleQRCodeCase> file_goal_handle_{ nullptr };
  std::shared_ptr<qrb::vision_manager::RequestManager> request_manager_{ nullptr };
  std::shared_ptr<rclcpp::Node> node_handler_{ nullptr };
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp_action::Server<QRCodeCase>::SharedPtr qr_case_server_;
  std::shared_ptr<std::thread> thread_;
};

}  // namespace qrb_ros::vision_service
#endif