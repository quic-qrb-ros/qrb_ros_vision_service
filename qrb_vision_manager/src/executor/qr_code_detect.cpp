// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#include "qrb_vision_manager/executor/qr_code_detect.hpp"

#include <zbar.h>

#include <fstream>

namespace qrb::vision_manager
{
QRCodeDetectExecutor::QRCodeDetectExecutor(std::shared_ptr<CallbackManager> & cb_manager,
    const std::string & camera_name)
  : ExecutorBase(cb_manager, 255), camera_name_(camera_name)
{
}

QRCodeDetectExecutor::QRCodeDetectExecutor(std::shared_ptr<CallbackManager> & cb_manager,
    const uint8_t camera_id)
  : ExecutorBase(cb_manager, camera_id), is_stream_(true)
{
  camera_name_ = "camera-" + std::to_string(camera_id);
}

QRCodeDetectExecutor::~QRCodeDetectExecutor()
{
  running_ = false;
  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
  }
  status_list_.clear();
}

bool QRCodeDetectExecutor::has_contain(const std::string contain)
{
  auto it = std::find(contains_.begin(), contains_.end(), contain);
  return it != contains_.end();
}

bool QRCodeDetectExecutor::load_image(const std::string & path, cv::Mat & mat)
{
  std::ifstream f(path.c_str());
  if (!f.good()) {
    cb_manager_->send_qrcode_result({}, "file path is invaild", false, camera_name_);
    return false;
  }
  mat = cv::imread(path);
  return true;
}

void QRCodeDetectExecutor::detect_image(cv::Mat & image)
{
  cv::Mat downscale_image;
  if (image.cols > 1280 && image.rows > 720) {
    float width_ratio = 1280.0f / image.cols;
    float height_ratio = 720.0f / image.rows;
    float min_ratio = width_ratio < height_ratio ? width_ratio : height_ratio;
    cv::resize(image, downscale_image, cv::Size(0, 0), min_ratio, min_ratio);
  } else {
    downscale_image = image;
  }

  cv::Mat gray;
  if (image.channels() != 1) {
    cvtColor(downscale_image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = downscale_image;
  }

  zbar::ImageScanner scanner;
  scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

  int width = gray.cols;
  int height = gray.rows;
  uchar * raw = (uchar *)gray.data;
  zbar::Image imageZbar(width, height, "Y800", raw, width * height);

  scanner.scan(imageZbar);

  std::vector<std::string> contents;
  // TODO: Update deduplication method
  for (auto symbol = imageZbar.symbol_begin(); symbol != imageZbar.symbol_end(); ++symbol) {
    if (symbol->get_type_name() == "QR-Code" && !has_contain(symbol->get_data())) {
      contains_.push_back(symbol->get_data());
      contents.push_back(symbol->get_data());
    }
  }
  if (!this->running_) {
    std::cout << "[QRCodeDetectExecutor]: send qr code result, source: " << camera_name_
              << std::endl;
    cb_manager_->send_qrcode_result(contents, "", true, camera_name_);
    if (!is_stream_) {
      cv_.notify_one();
    }
  } else if (!contents.empty()) {
    cb_manager_->send_qrcode_feedback(contents, camera_name_);
  }
}

bool QRCodeDetectExecutor::execute(const std::vector<std::string> & files)
{
  running_ = true;
  auto fun = [this, &files]() -> void {
    auto it = files.begin();
    while (files.size() > 1 && (it != files.end() - 1)) {
      bool running;
      {
        std::unique_lock<std::mutex> lock(mtx_);
        running = this->running_;
      }
      if (!running) {
        return;
      }
      cv::Mat mat;
      if (!load_image(*it, mat)) {
        return;
      }
      detect_image(mat);
      it++;
    }
    cv::Mat mat;
    if (!load_image(*it, mat)) {
      return;
    }
    this->running_ = false;
    detect_image(mat);
  };
  thread_ = std::make_shared<std::thread>(fun);
  return true;
}

bool QRCodeDetectExecutor::execute(const uint8_t camera_id, CameraStatus & cam_status)
{
  running_ = true;

  std::vector<ComponentInfo> component_infos;

  bool need_resize = false;
  ComponentInfo camera_info;
  int camera_height, camera_width;

  bool ret = get_camera_node_info(camera_info, need_resize, camera_height, camera_width);
  if (!ret) {
    cb_manager_->send_qrcode_result(
        {}, "please add camera config in config file", false, camera_name_);
    return false;
  }
  component_infos.push_back(camera_info);

  ComponentInfo convert_info;
  ret = get_color_convert_node_info(convert_info);
  if (!ret) {
    cb_manager_->send_qrcode_result(
        {}, "please add color convert config in config file", false, camera_name_);
    return false;
  }
  component_infos.push_back(convert_info);

  std::string topic_name;
  int pipeline;
  ret = cb_manager_->send_pipeline_request(TaskCommand::PIPELINE_CREATE, pipeline_id_,
      component_infos, std::vector<int>(), topic_name, pipeline);
  if (!ret) {
    cb_manager_->send_qrcode_result({}, "create pipeline failed", false, camera_name_);
    return false;
  }
  pipeline_id_ = pipeline;
  ret = cb_manager_->send_pipeline_request(TaskCommand::PIPELINE_LOAD, pipeline_id_,
      component_infos, std::vector<int>(), topic_name, pipeline);
  std::cout << "[QRCodeDetectExecutor]: send pipeline load request" << std::endl;
  if (!ret) {
    cb_manager_->send_qrcode_result({}, "load pipeline failed", false, camera_name_);
    return false;
  }

  using namespace std::placeholders;
  cb_manager_->send_create_sub_image(
      camera_name_, topic_name, std::bind(&QRCodeDetectExecutor::detect_image, this, _1));

  cam_status.camera_id = camera_id_;
  cam_status.topic_name = topic_name;
  cam_status.call_number = 1;
  cam_status.height = camera_height;
  cam_status.width = camera_width;
  cam_status.pipeline_id = pipeline_id_;
  return true;
}

bool QRCodeDetectExecutor::stop_case(bool stop_camera)
{
  if (!running_) {
    std::cout << "[QRCodeDetectExecutor]: please call execute first or the file detection has "
                 "already detected the last one."
              << std::endl;
    return false;
  }

  {
    std::unique_lock<std::mutex> lock(mtx_);
    this->running_ = false;
  }

  // stop read file case & topic case
  if (!is_stream_) {
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock);
    return true;
  }

  // delete whole pipeline case
  std::vector<int> ids;
  for (int i = 1; i < status_list_.size(); i++) {
    ids.push_back(status_list_[i].unique_id);
  }

  bool ret;

  std::string topic_name;
  int pipeline;
  std::vector<ComponentInfo> info;

  if (stop_camera) {
    // delete the whole pipeline
    ret = cb_manager_->send_pipeline_request(
        TaskCommand::PIPELINE_DESTROY, pipeline_id_, info, ids, topic_name, pipeline);
  } else {
    // remove other unique id
    ret = cb_manager_->send_pipeline_request(
        TaskCommand::PIPELINE_UNLOAD, pipeline_id_, info, ids, topic_name, pipeline);
  }
  if (ret) {
    status_list_.clear();
  }

  return ret;
}

bool QRCodeDetectExecutor::execute()
{
  running_ = true;

  using namespace std::placeholders;
  cb_manager_->send_create_sub_image(
      camera_name_, camera_name_, std::bind(&QRCodeDetectExecutor::detect_image, this, _1));
  return true;
}

}  // namespace qrb::vision_manager
