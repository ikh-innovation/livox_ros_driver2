//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <iostream>
#include <chrono>
#include <vector>
#include <csignal>
#include <thread>

#include "include/livox_ros_driver2.h"
#include "include/ros_headers.h"
#include "driver_node.h"
#include "lddc.h"
#include "lds_lidar.h"

using namespace livox_ros;

void DriverNode::onInit()
{
  /** Ros related */
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();

  DRIVER_INFO(this, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq  = 10.0; /* Hz */
  int output_type      = kOutputToRos;
  std::string frame_id = "livox_frame";
  bool lidar_bag = true;
  bool imu_bag   = false;
  bool sample_at_startup = false;

  private_nh.getParam("xfer_format", xfer_format);
  private_nh.getParam("multi_topic", multi_topic);
  private_nh.getParam("data_src", data_src);
  private_nh.getParam("publish_freq", publish_freq);
  private_nh.getParam("output_data_type", output_type);
  private_nh.getParam("frame_id", frame_id);
  private_nh.getParam("enable_lidar_bag", lidar_bag);
  private_nh.getParam("enable_imu_bag", imu_bag);
  private_nh.getParam("sample_at_startup", sample_at_startup);

  printf("data source:%u.\n", data_src);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  } else {
    publish_freq = publish_freq;
  }

  future_ = exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type, publish_freq, frame_id, lidar_bag, imu_bag);
  lddc_ptr_->SetRosNode(this);

  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(this, "Data Source is raw lidar.");

    std::string user_config_path;
    private_nh.getParam("user_config_path", user_config_path);
    DRIVER_INFO(this, "Config file : %s", user_config_path.c_str());

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq, sample_at_startup);
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(this, "Init lds lidar successfully!");
    } else {
      DRIVER_ERROR(this, "Init lds lidar failed!");
    }
  } else {
    DRIVER_ERROR(this, "Invalid data src (%d), please check the launch file", data_src);
  }

  pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, this);
  imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, this);
  stateinfo_poll_thread_ = std::make_shared<std::thread>(&DriverNode::StateInfoPollThread, this);

  // Get state info topics that are used for the enable_sampling service
  ros::Duration(5.0).sleep();
  while (state_topics_.empty())
  {
    state_topics_ = lddc_ptr_->GetStateTopics();
    ros::Duration(1.0).sleep();
  }
  
  sampling_service_ = private_nh.advertiseService("livox/enable_sampling", &DriverNode::SetSamplingCallback, this);
}
PLUGINLIB_EXPORT_CLASS(livox_ros::DriverNode,nodelet::Nodelet)


void DriverNode::PointCloudDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributePointCloudData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

void DriverNode::ImuDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributeImuData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

void DriverNode::StateInfoPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributeStateInfo();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

bool DriverNode::SetSamplingCallback(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res)
{
  LivoxLidarWorkMode desired_mode{req.data ? kLivoxLidarNormal : kLivoxLidarWakeUp};
  
  {
    std::unique_lock<std::mutex> lock(mtx_);
    callbacks_status_ = true;
    callbacks_done_ = 0;
  }
  
  uint8_t actual_lidar_count{0};

  for (int i = 0; i < lddc_ptr_->lds_->lidar_count_; i++) 
  {
    LidarDevice * p_lidar = &(lddc_ptr_->lds_->lidars_[i]);
    if (p_lidar->lidar_type & kLivoxLidarType) 
    {
      uint32_t handle = p_lidar->handle;
      actual_lidar_count++;
      SetLivoxLidarWorkMode(handle, desired_mode, DriverNode::WorkModeChangeOnceCallback, this);
    }
  }

  // Wait for callbacks to finish
  {
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock, [this, actual_lidar_count]{ return (callbacks_done_ == actual_lidar_count); });
    callbacks_done_ = 0;
  }

  if (req.data && callbacks_status_)
  {
    // Subscribe to state info topics
    if (state_topics_.size() != actual_lidar_count)
    {
      DRIVER_WARN(*this, "State topics vector has different size than the actual_lidar_count! Taking the minimum.");
      actual_lidar_count = std::min(static_cast<uint8_t>(state_topics_.size()), actual_lidar_count);
    }
    for (int i = 0; i < actual_lidar_count; i++)
    {
      const std::string& topic_name = state_topics_[i];
      state_subs_.push_back(getMTNodeHandle().subscribe<livox_ros_driver2::StateInfoMsg>(topic_name, 1, [this, i](const livox_ros_driver2::StateInfoMsgConstPtr& msg){this->state_cb(msg, i);}));
    }
    
    // Wait for callbacks to finish
    {
      std::unique_lock<std::mutex> lock(mtx_);
      res.success = cv_.wait_for(lock, std::chrono::seconds(wait_timeout_), [this, actual_lidar_count]{ return (callbacks_done_ == actual_lidar_count);});
    }

    // Destroy subscribers
    state_subs_.clear();

    res.message = res.success ? "Work mode changed successfully" : "Timed out while waiting for state transition";
  }
  else
  {
    res.success = callbacks_status_;
    res.message = res.success ? "Work mode changed successfully" : "Failed to change work mode";
  }

  return true;
}

void DriverNode::state_cb(const livox_ros_driver2::StateInfoMsgConstPtr& msg, int sub_index)
{
  if (msg->work_tgt_mode == kLivoxLidarNormal && msg->cur_work_state == kLivoxLidarNormal)
  {
    std::unique_lock<std::mutex> lock(mtx_);
    callbacks_done_ = callbacks_done_ + 1;
    state_subs_[sub_index].shutdown();
    cv_.notify_one();
  }
}

void DriverNode::WorkModeChangeOnceCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) 
{
  DriverNode* node = static_cast<DriverNode*>(client_data);
  {
    std::unique_lock<std::mutex> lock(node->mtx_);
    if (status != kLivoxLidarStatusSuccess) 
    {
      node->callbacks_status_ = false;
    }
    node->callbacks_done_ = node->callbacks_done_ + 1;
  }
  node->cv_.notify_one();
}