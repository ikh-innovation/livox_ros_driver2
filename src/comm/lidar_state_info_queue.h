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

#ifndef LIVOX_ROS_DRIVER_LIDAR_STATE_INFO_QUEUE_H_
#define LIVOX_ROS_DRIVER_LIDAR_STATE_INFO_QUEUE_H_

#include <list>
#include <mutex>
#include <cstdint>

namespace livox_ros {

struct IpConfig {
  std::string ip_addr;
  std::string net_mask;
  std::string gw_addr;
};

struct HostIpConfig {
  std::string ip_addr;
  uint32_t dst_port;
  uint32_t src_port;
};

struct InstallAttitude {
  double roll_deg;
  double pitch_deg;
  double yaw_deg;
  uint32_t x;
  uint32_t y;
  uint32_t z;
};

struct FovConfig {
  int yaw_start;
  int yaw_stop;
  int pitch_start;
  int pitch_stop;
};

struct LidarInfo {
  uint32_t pcl_data_type;
  uint32_t pattern_mode;
  // uint32_t dual_emit_en;
  // uint32_t point_send_en;
  IpConfig lidar_ipcfg;
  HostIpConfig info_host_ipcfg;
  HostIpConfig pointcloud_host_ipcfg;
  HostIpConfig imu_host_ipcfg;
  // HostIpConfig ctl_host_ipcfg;
  // HostIpConfig log_host_ipcfg;
  // int vehicle_speed;
  // int environment_temp;
  InstallAttitude install_attitude;
  // uint32_t blind_spot_set;
  // uint32_t frame_rate;
  FovConfig fov_cfg0;
  FovConfig fov_cfg1;
  uint32_t fov_cfg_en;
  uint32_t detect_mode;
  std::array<uint32_t, 4> func_io_cfg;
  uint32_t work_tgt_mode;
  // uint32_t glass_heat;
  uint32_t imu_data_en;
  // uint32_t fusa_en;
  std::string sn;
  std::string product_info;
  std::array<uint32_t, 4> version_app;
  std::array<uint32_t, 4> version_loader;
  std::array<uint32_t, 4> version_hardware;
  std::array<uint32_t, 6> mac;
  uint32_t cur_work_state;
  int core_temp;
  uint32_t powerup_cnt;
  uint64_t local_time_now;
  uint64_t last_sync_time;
  int64_t time_offset;
  uint32_t time_sync_type;
  // std::array<uint32_t, 32> status_code;
  uint32_t lidar_diag_status;
  // uint32_t lidar_flash_status;
  uint32_t fw_type;
  std::array<uint32_t, 8> hms_code;
  // uint32_t ROI_Mode;
};

typedef struct {
  uint8_t lidar_type;
  uint32_t handle;
  LidarInfo data;
} StateInfo;

class LidarStateInfoQueue {
 public:
  void Push(StateInfo* state_info);
  bool Pop(StateInfo& state_info);
  bool Empty();
  void Clear();

 private:
  std::mutex mutex_;
  std::list<StateInfo> state_info_queue_;
};

} // namespace

#endif // LIVOX_ROS_DRIVER_LIDAR_STATE_INFO_QUEUE_H_

