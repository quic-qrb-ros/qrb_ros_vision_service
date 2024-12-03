/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include <memory>

#include "qrb_ros_vision_service/vision_service.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto service = std::make_shared<qrb_ros::vision_service::VisionService>();
  service->run();
  rclcpp::shutdown();
  return 0;
}