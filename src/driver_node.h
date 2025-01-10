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

#ifndef LIVOX_DRIVER_NODE_H
#define LIVOX_DRIVER_NODE_H

#include "include/ros_headers.h"
#include "livox_lidar_def.h"
#include <std_srvs/SetBool.h>

namespace livox_ros {

class Lddc;

#ifdef BUILDING_ROS1
class DriverNode final : public ros::NodeHandle {
 public:
  DriverNode() = default;
  DriverNode(const DriverNode &) = delete;
  ~DriverNode();
  DriverNode &operator=(const DriverNode &) = delete;

  DriverNode& GetNode() noexcept;

  void PointCloudDataPollThread();
  void ImuDataPollThread();
  void StateInfoPollThread();
  bool SetSamplingCallback(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res);
  static void WorkModeChangeOnceCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data);

  std::unique_ptr<Lddc> lddc_ptr_;
  std::shared_ptr<std::thread> pointclouddata_poll_thread_;
  std::shared_ptr<std::thread> imudata_poll_thread_;
  std::shared_ptr<std::thread> stateinfo_poll_thread_;
  ros::ServiceServer sampling_service_;
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;
  std::condition_variable cv_;
  std::mutex mtx_;
  uint callbacks_done_;
  bool callbacks_status_;
};

#elif defined BUILDING_ROS2
class DriverNode final : public rclcpp::Node {
 public:
  explicit DriverNode(const rclcpp::NodeOptions& options);
  DriverNode(const DriverNode &) = delete;
  ~DriverNode();
  DriverNode &operator=(const DriverNode &) = delete;

  DriverNode& GetNode() noexcept;

 private:
  void PointCloudDataPollThread();
  void ImuDataPollThread();

  std::unique_ptr<Lddc> lddc_ptr_;
  std::shared_ptr<std::thread> pointclouddata_poll_thread_;
  std::shared_ptr<std::thread> imudata_poll_thread_;
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;
};
#endif

} // namespace livox_ros

#endif // LIVOX_DRIVER_NODE_H