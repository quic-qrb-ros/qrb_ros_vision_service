// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#ifndef QRB_VISION_MANAGER__EXECUTOR__EXECUTOR_BASE_HPP_
#define QRB_VISION_MANAGER__EXECUTOR__EXECUTOR_BASE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "qrb_vision_manager/callback_manager.hpp"
#include "qrb_vision_manager/camera_manager.hpp"
#include "qrb_vision_manager/parameter_manager.hpp"

namespace qrb::vision_manager
{
class TaskCommand
{
public:
  const static int REGISTER_CAMERA = 101;
  const static int UNREGISTER_CAMERA = 102;
  const static int REGISTER_VIDEO = 201;
  const static int UNREGISTER_VIDEO = 202;
  const static int REGISTER_OBJECT_DETECT = 301;
  const static int UNREGISTER_OBJECT_DETECT = 302;
  const static int REGISTER_IMAGE_SEGEMENT = 401;
  const static int UNREGISTER_IMAGE_SEGEMENT = 402;

  const static int REGISTER_CAMERA_QR_DETECT = 1;
  const static int START_QR_CODE_CASE_FILES = 2;
  const static int UNREGISTER_CAMERA_QR_DETECT = 3;
  const static int STOP_FILE_CASE = 4;
  const static int REGISTER_TOPIC_QR_DETECT = 5;
  const static int UNREGISTER_TOPIC_QR_DETECT = 6;

  const static int PIPELINE_CREATE = 257;
  const static int PIPELINE_DESTROY = 258;
  const static int PIPELINE_LOAD = 259;
  const static int PIPELINE_UNLOAD = 3;
};

class ExecutorBase
{
public:
  ExecutorBase(std::shared_ptr<CallbackManager> & cb_manager, const uint8_t camera_id);

  virtual ~ExecutorBase(){};

  const uint8_t get_camera_id() const { return camera_id_; }

  virtual bool enable(int height, int width, CameraStatus & cam_status) { return false; }

  virtual bool stop(int height = 0, int width = 0, bool stop_camera = false) { return false; }

  virtual void add_new_listener(int height, int width) {}

  const static std::string CAMERA;
  const static std::string VIDEO;
  const static std::string OBJECT_DETECT;
  const static std::string IMAGE_SEGEMENT;
  const static std::string QRCODE_DETECT;

protected:
  virtual const std::string get_topic_name(int height, int width) { return ""; }

  bool get_camera_node_info(ComponentInfo & component_info,
      bool & need_resize,
      int & camera_height,
      int & camera_width,
      int require_height = 0,
      int require_width = 0);

  bool get_resize_node_info(ComponentInfo & component_info, int require_height, int require_width);

  // require_type: 0 means nv12_to_rgb8, 1 means rgb8_to_nv12
  bool get_color_convert_node_info(ComponentInfo & component_info, int require_type = 0);

  bool get_yolo_pre_process_node_info(ComponentInfo & component_info);

  bool get_yolo_object_det_post_process_node_info(ComponentInfo & component_info);

  bool get_yolo_image_seg_post_process_node_info(ComponentInfo & component_info);

  bool get_nn_node_info(ComponentInfo & component_info, int require_type);

  bool get_video_node_info(ComponentInfo & component_info, int require_height, int require_width);

  bool get_parameter_from_node_info(const std::vector<std::string> & param_name_list,
      const std::string & parameter_name,
      int & index);

  bool add_parameter_to_component_info(const std::vector<std::string> & param_name_list,
      const std::vector<std::vector<std::string>> & param_list,
      const std::string & param_name,
      std::vector<std::string> & parameters,
      int reindex = 0);

  bool get_parameter_to_component_info(const std::vector<std::string> & param_name_list,
      const std::vector<std::vector<std::string>> & param_list,
      const std::string & param_name,
      std::string & param_output);

  std::shared_ptr<ParameterManager> parameter_manager_;
  std::shared_ptr<CallbackManager> cb_manager_;
  std::vector<ComponentStatus> status_list_;
  int listener_;
  uint8_t camera_id_;
  int pipeline_id_{ -1 };
};
}  // namespace qrb::vision_manager
#endif
