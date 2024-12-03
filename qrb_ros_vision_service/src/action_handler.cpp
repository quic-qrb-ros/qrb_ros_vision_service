/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "qrb_ros_vision_service/action_handler.hpp"

#include "cv_bridge/cv_bridge.h"

using namespace qrb::vision_manager;

namespace qrb_ros::vision_service
{
ActionHandler::ActionHandler(std::shared_ptr<rclcpp::Node> node_handler,
    std::shared_ptr<RequestManager> require_manager)
  : node_handler_(node_handler), request_manager_(require_manager)
{
  using namespace std::placeholders;
  callback_group_ = node_handler_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto options = rcl_action_server_get_default_options();
  qr_case_server_ = rclcpp_action::create_server<QRCodeCase>(
      node_handler_->get_node_base_interface(), node_handler_->get_node_clock_interface(),
      node_handler_->get_node_logging_interface(), node_handler_->get_node_waitables_interface(),
      "qr_case", std::bind(&ActionHandler::handle_goal, this, _1, _2),
      std::bind(&ActionHandler::handle_cancel, this, _1),
      std::bind(&ActionHandler::handle_accepted, this, _1), options, callback_group_);
  request_manager_->register_qrcode_feedback_callback(
      std::bind(&ActionHandler::qr_code_feedback, this, _1, _2));
  request_manager_->register_qrcode_result_callback(
      std::bind(&ActionHandler::qr_code_result, this, _1, _2, _3, _4));
  request_manager_->register_create_image_sub_callback(
      std::bind(&ActionHandler::create_image_sub, this, _1, _2, _3));
}

ActionHandler::~ActionHandler()
{
  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
  }
}

bool ActionHandler::remove_goal_handle(
    std::vector<std::shared_ptr<GoalHandleQRCodeCase>> & goal_handle_list,
    const std::shared_ptr<GoalHandleQRCodeCase> goal_handle)
{
  auto it = std::find(goal_handle_list.begin(), goal_handle_list.end(), goal_handle);
  if (it != goal_handle_list.end()) {
    if (goal_handle_list.size() == 1) {
      // can stop the case
      request_manager_->execute_qr_case(
          QRCodeCase::Goal::UNREGISTER_CAMERA_QR_DETECT, goal_handle->get_goal()->camera_id);
    } else {
      goal_handle_list.erase(it);
    }
    return true;
  }
  return false;
}

bool ActionHandler::is_case_working(const std::string & name)
{
  return goal_handle_map_.find(name) != goal_handle_map_.end();
}

void ActionHandler::remove_file_handler()
{
  std::unique_lock<std::mutex> lock(mtx_);
  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
  }
  file_goal_handle_ = nullptr;
  auto fun = [this]() -> void {
    request_manager_->execute_qr_case(QRCodeCase::Goal::STOP_FILE_CASE, 0);
  };
  thread_ = std::make_shared<std::thread>(fun);
}

rclcpp_action::GoalResponse ActionHandler::handle_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const QRCodeCase::Goal> goal)
{
  std::unique_lock<std::mutex> lock(mtx_);
  if (goal->action_id == QRCodeCase::Goal::REGISTER_CAMERA_QR_DETECT ||
      goal->action_id == QRCodeCase::Goal::REGISTER_TOPIC_QR_DETECT) {
    // qr stream case
    RCLCPP_INFO(node_handler_->get_logger(), "receive start stream action goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  if (goal->action_id == QRCodeCase::Goal::START_FILE_CASE && file_goal_handle_ == nullptr) {
    // detect file
    RCLCPP_INFO(node_handler_->get_logger(), "receive start file action goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  if ((goal->action_id == QRCodeCase::Goal::UNREGISTER_CAMERA_QR_DETECT &&
          is_case_working("camera-" + std::to_string(goal->camera_id))) ||
      goal->action_id == QRCodeCase::Goal::STOP_FILE_CASE ||
      (goal->action_id == QRCodeCase::Goal::UNREGISTER_TOPIC_QR_DETECT &&
          is_case_working(goal->topic_name))) {
    RCLCPP_INFO(node_handler_->get_logger(), "receive stop goal, %d", goal->action_id);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  RCLCPP_ERROR(
      node_handler_->get_logger(), "Can not execute this request goal due to camera not available");
  return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse ActionHandler::handle_cancel(
    const std::shared_ptr<GoalHandleQRCodeCase> goal_handle)
{
  RCLCPP_INFO(node_handler_->get_logger(), "receive cancel goal");
  const auto goal = goal_handle->get_goal();
  if (goal->action_id == QRCodeCase::Goal::REGISTER_CAMERA_QR_DETECT &&
      is_case_working("camera-" + std::to_string(goal->camera_id))) {
    RCLCPP_INFO(node_handler_->get_logger(), "receive cancel stream goal");
    auto it = goal_handle_map_.find("camera-" + std::to_string(goal->camera_id));
    if (remove_goal_handle(it->second, goal_handle)) {
      return rclcpp_action::CancelResponse::ACCEPT;
    }
  }
  if (goal->action_id == QRCodeCase::Goal::START_FILE_CASE && goal_handle == file_goal_handle_) {
    RCLCPP_INFO(node_handler_->get_logger(), "receive cancel file goal");
    request_manager_->execute_qr_case(QRCodeCase::Goal::STOP_FILE_CASE, 0);
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  if (goal->action_id == QRCodeCase::Goal::REGISTER_TOPIC_QR_DETECT &&
      is_case_working(goal->topic_name)) {
    RCLCPP_INFO(node_handler_->get_logger(), "receive cancel topic detect goal");
    std::string name = goal->topic_name;
    auto it = goal_handle_map_.find(name);
    if (remove_goal_handle(it->second, goal_handle)) {
      return rclcpp_action::CancelResponse::ACCEPT;
    }
  }
  RCLCPP_INFO(node_handler_->get_logger(), "receive cancel goal but dont executed");
  return rclcpp_action::CancelResponse::REJECT;
}

void ActionHandler::handle_accepted(const std::shared_ptr<GoalHandleQRCodeCase> goal_handle)
{
  using namespace std::placeholders;
  RCLCPP_INFO(node_handler_->get_logger(), "Executing action goal");
  const auto goal = goal_handle->get_goal();
  if (goal->action_id == QRCodeCase::Goal::REGISTER_CAMERA_QR_DETECT) {
    std::string name = "camera-" + std::to_string(goal->camera_id);
    auto it = goal_handle_map_.find(name);
    if (it != goal_handle_map_.end()) {
      it->second.push_back(goal_handle);
      return;
    }
    std::vector<std::shared_ptr<GoalHandleQRCodeCase>> goal_handle_list;
    goal_handle_list.push_back(goal_handle);
    goal_handle_map_.insert(std::make_pair(name, goal_handle_list));
    std::thread{ std::bind(&ActionHandler::execute_stream_detect, this, _1), goal_handle }.detach();
  } else if (goal->action_id == QRCodeCase::Goal::REGISTER_TOPIC_QR_DETECT) {
    std::string name = goal->topic_name;
    auto it = goal_handle_map_.find(name);
    if (it != goal_handle_map_.end()) {
      it->second.push_back(goal_handle);
      return;
    }
    std::vector<std::shared_ptr<GoalHandleQRCodeCase>> goal_handle_list;
    goal_handle_list.push_back(goal_handle);
    goal_handle_map_.insert(std::make_pair(name, goal_handle_list));
    request_manager_->execute_qr_case(goal->action_id, goal->topic_name);
  } else if (goal->action_id == QRCodeCase::Goal::START_FILE_CASE) {
    std::thread{ std::bind(&ActionHandler::execute_file_detect, this, _1), goal_handle }.detach();
  } else if (goal->action_id == QRCodeCase::Goal::UNREGISTER_CAMERA_QR_DETECT ||
             goal->action_id == QRCodeCase::Goal::STOP_FILE_CASE ||
             goal->action_id == QRCodeCase::Goal::UNREGISTER_TOPIC_QR_DETECT) {
    execute_stop_detect(goal_handle);
  }
}

void ActionHandler::execute_stream_detect(const std::shared_ptr<GoalHandleQRCodeCase> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  request_manager_->execute_qr_case(goal->action_id, goal->camera_id);
}

void ActionHandler::execute_file_detect(const std::shared_ptr<GoalHandleQRCodeCase> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  if (goal->file_path.empty()) {
    auto result = std::make_shared<QRCodeCase::Result>();
    result->success = false;
    result->error_info = "this case is not executed or stop failed";
    goal_handle->succeed(result);
    return;
  }
  request_manager_->execute_qr_case(goal->action_id, goal->file_path);
  file_goal_handle_ = goal_handle;
}

void ActionHandler::execute_stop_detect(const std::shared_ptr<GoalHandleQRCodeCase> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  bool ret;
  if (goal->action_id == QRCodeCase::Goal::UNREGISTER_CAMERA_QR_DETECT ||
      goal->action_id == QRCodeCase::Goal::STOP_FILE_CASE) {
    ret = request_manager_->execute_qr_case(goal->action_id, goal->camera_id);
  } else {
    ret = request_manager_->execute_qr_case(goal->action_id, goal->topic_name);
  }
  auto result = std::make_shared<QRCodeCase::Result>();
  result->success = ret;
  if (!ret) {
    result->error_info = "this case is not executed or stop failed";
  } else {
    std::string msg = "stop success";
    result->contents.push_back(msg);
  }
  goal_handle->succeed(result);
}

void ActionHandler::qr_code_feedback(const std::vector<std::string> & contents,
    const std::string & source)
{
  RCLCPP_INFO(node_handler_->get_logger(), "receive qr code feedback");
  auto feedback = std::make_shared<QRCodeCase::Feedback>();
  feedback->contents.resize(contents.size());
  std::copy(contents.begin(), contents.end(), feedback->contents.begin());
  // file feedback
  if (file_goal_handle_ != nullptr && source == "file") {
    if (file_goal_handle_->is_canceling()) {
      auto result = std::make_shared<QRCodeCase::Result>();
      result->error_info = "user cancel the goal";
      result->contents.resize(contents.size());
      std::copy(contents.begin(), contents.end(), result->contents.begin());
      result->success = false;
      file_goal_handle_->canceled(result);
      remove_file_handler();
      return;
    }
    file_goal_handle_->publish_feedback(feedback);
    return;
  }

  // stream feedback
  auto handle_it = goal_handle_map_.find(source);
  if (handle_it == goal_handle_map_.end()) {
    RCLCPP_ERROR(node_handler_->get_logger(), "can not find %s handler", source.c_str());
    return;
  }
  for (auto it = handle_it->second.begin(); it != handle_it->second.end();) {
    auto handler = (*it);
    if (handler->is_canceling()) {
      RCLCPP_INFO(node_handler_->get_logger(), "%s handler is canceling", source.c_str());
      auto result = std::make_shared<QRCodeCase::Result>();
      result->error_info = "user cancel the goal";
      result->contents.resize(contents.size());
      std::copy(contents.begin(), contents.end(), result->contents.begin());
      result->success = false;
      handler->canceled(result);
      std::unique_lock<std::mutex> lock(mtx_);
      if (handle_it->second.size() == 1) {
        goal_handle_map_.erase(handle_it);
        auto sub_it = image_sub_map_.find(source);
        image_sub_map_.erase(sub_it);
        break;
      }
      handle_it->second.erase(it);
      RCLCPP_INFO(node_handler_->get_logger(), "QR detect goal canceled");
      continue;
    }
    handler->publish_feedback(feedback);
    it++;
  }
}

void ActionHandler::qr_code_result(const std::vector<std::string> & contents,
    const std::string & err_info,
    bool success,
    const std::string & source)
{
  RCLCPP_INFO(node_handler_->get_logger(), "%s receive qr code result.", source.c_str());
  auto result = std::make_shared<QRCodeCase::Result>();
  result->error_info = err_info;
  result->contents.resize(contents.size());
  std::copy(contents.begin(), contents.end(), result->contents.begin());
  result->success = success;
  // file feedback
  if (file_goal_handle_ != nullptr && source == "file") {
    file_goal_handle_->succeed(result);
    remove_file_handler();
    return;
  }

  // stream feedback
  auto handle_it = goal_handle_map_.find(source);

  // can not find this handler
  if (handle_it == goal_handle_map_.end()) {
    RCLCPP_INFO(node_handler_->get_logger(), "can not find this handler: %s", source.c_str());
    return;
  }
  for (auto handler : handle_it->second) {
    handler->succeed(result);
  }
  std::unique_lock<std::mutex> lock(mtx_);
  goal_handle_map_.erase(handle_it);
  auto sub_it = image_sub_map_.find(source);
  image_sub_map_.erase(sub_it);
}

void ActionHandler::create_image_sub(const std::string & source,
    const std::string & name,
    ImageCallback cb)
{
  RCLCPP_INFO(node_handler_->get_logger(), "receive image sub: %s", name.c_str());
  auto topic_callback = [this, cb](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
    RCLCPP_DEBUG(this->node_handler_->get_logger(), "receive image callback");
    auto mat = cv_bridge::toCvShare(msg, msg->encoding)->image;
    cb(mat);
  };
  auto image_sub =
      node_handler_->create_subscription<sensor_msgs::msg::Image>(name, 10, topic_callback);
  std::unique_lock<std::mutex> lock(mtx_);
  image_sub_map_.insert(std::make_pair(source, image_sub));
}

}  // namespace qrb_ros::vision_service
