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

#include "lds_lidar.h"

#include <stdio.h>
#include <string.h>
#include <memory>
#include <mutex>
#include <thread>

#ifdef WIN32
#include <winsock2.h>
#include <ws2def.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif // WIN32

#include "comm/comm.h"
#include "comm/pub_handler.h"

#include "parse_cfg_file/parse_cfg_file.h"
#include "parse_cfg_file/parse_livox_lidar_cfg.h"

#include "call_back/lidar_common_callback.h"
#include "call_back/livox_lidar_callback.h"

using namespace std;

namespace livox_ros {

/** Lds lidar function -------------------------------------------------------*/
LdsLidar::LdsLidar(double publish_freq, const bool start_at_startup)
    : Lds(publish_freq, start_at_startup, kSourceRawLidar), 
      auto_connect_mode_(true),
      whitelist_count_(0),
      is_initialized_(false) {
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));
  ResetLdsLidar();
}

LdsLidar::~LdsLidar() {}

void LdsLidar::ResetLdsLidar(void) { ResetLds(kSourceRawLidar); }

bool LdsLidar::InitLdsLidar(const std::string& path_name) {
  if (is_initialized_) {
    printf("Lds is already inited!\n");
    return false;
  }

  path_ = path_name;
  if (!InitLidars()) {
    return false;
  }
  SetLidarPubHandle();
  if (!Start()) {
    return false;
  }
  is_initialized_ = true;
  return true;
}

bool LdsLidar::InitLidars() {
  if (!ParseSummaryConfig()) {
    return false;
  }
  std::cout << "config lidar type: " << static_cast<int>(lidar_summary_info_.lidar_type) << std::endl;

  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (!InitLivoxLidar()) {
      return false;
    }
  }
  return true;
}


bool LdsLidar::Start() {
  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (!LivoxLidarStart()) {
      return false;
    }
  }
  return true;
}

bool LdsLidar::ParseSummaryConfig() {
  return ParseCfgFile(path_).ParseSummaryInfo(lidar_summary_info_);
}

bool LdsLidar::InitLivoxLidar() {
  
  DisableLivoxSdkConsoleLogger();

  // parse user config
  LivoxLidarConfigParser parser(path_);
  std::vector<UserLivoxLidarConfig> user_configs;
  if (!parser.Parse(user_configs)) {
    std::cout << "failed to parse user-defined config" << std::endl;
  }

  // SDK initialization
  if (!LivoxLidarSdkInit(path_.c_str())) {
    std::cout << "Failed to init livox lidar sdk." << std::endl;
    return false;
  }

  // fill in lidar devices
  uint8_t actual_lidar_count{0};
  for (auto& config : user_configs) {
    uint8_t index = 0;
    int8_t ret = cache_index_.GetFreeIndex(kLivoxLidarType, config.handle, index);
    if (ret != 0) {
      std::cout << "failed to get free index, lidar ip: " << IpNumToString(config.handle) << std::endl;
      continue;
    }
    LidarDevice *p_lidar = &(lidars_[index]);
    p_lidar->lidar_type = kLivoxLidarType;
    p_lidar->livox_config = config;
    p_lidar->handle = config.handle;
    actual_lidar_count++;


    LidarExtParameter lidar_param;
    lidar_param.handle = config.handle;
    lidar_param.lidar_type = kLivoxLidarType;
    if (config.pcl_data_type == kLivoxLidarCartesianCoordinateLowData) {
      // temporary resolution
      lidar_param.param.roll  = config.extrinsic_param.roll;
      lidar_param.param.pitch = config.extrinsic_param.pitch;
      lidar_param.param.yaw   = config.extrinsic_param.yaw;
      lidar_param.param.x     = config.extrinsic_param.x / 10;
      lidar_param.param.y     = config.extrinsic_param.y / 10;
      lidar_param.param.z     = config.extrinsic_param.z / 10;
    } else {
      lidar_param.param.roll  = config.extrinsic_param.roll;
      lidar_param.param.pitch = config.extrinsic_param.pitch;
      lidar_param.param.yaw   = config.extrinsic_param.yaw;
      lidar_param.param.x     = config.extrinsic_param.x;
      lidar_param.param.y     = config.extrinsic_param.y;
      lidar_param.param.z     = config.extrinsic_param.z;
    }
    pub_handler_.AddLidarsExtParam(lidar_param);
  }

  // Reboot Lidar
  reboots_started_ = 0;
  SetLivoxLidarInfoChangeCallback(LivoxLidarCallback::LidarInfoChangeRebootCallback, this);
  // Wait for reboots to start taking place. Bounded with a timeout so that a lidar
  // which never comes online (unreachable, misconfigured IP, ...) cannot block this
  // call forever - that would in turn block onInit()/nodelet load or unload indefinitely.
  {
    constexpr auto kRebootWaitTimeout = std::chrono::seconds(30);
    std::unique_lock<std::mutex> lock(reboot_mutex_);
    bool all_rebooted = reboot_cv_.wait_for(lock, kRebootWaitTimeout,
        [this, actual_lidar_count]{ return (reboots_started_ == actual_lidar_count); });
    if (!all_rebooted) {
      std::cout << "Timed out waiting for lidar reboots to start (" << reboots_started_
                << "/" << static_cast<int>(actual_lidar_count) << " observed); aborting lidar init."
                << std::endl;
      LivoxLidarSdkUninit();
      return false;
    }
  }

  LivoxLidarSdkUninit();
  std::this_thread::sleep_for(std::chrono::seconds(12));

  // SDK second initialization
  if (!LivoxLidarSdkInit(path_.c_str())) {
    std::cout << "Failed to init livox lidar sdk." << std::endl;
    return false;
  }
  
  // Setup Lidar
  SetLivoxLidarInfoChangeCallback(LivoxLidarCallback::LidarInfoChangeCallback, this);
  
  return true;
}

void LdsLidar::SetLidarPubHandle() {
  pub_handler_.SetPointCloudsCallback(LidarCommonCallback::OnLidarPointClounCb, this);
  pub_handler_.SetImuDataCallback(LidarCommonCallback::LidarImuDataCallback, this);
  pub_handler_.SetStateInfoCallback(LidarCommonCallback::LidarStateInfoCallback, this);

  double publish_freq = Lds::GetLdsFrequency();
  pub_handler_.SetPointCloudConfig(publish_freq);
}

bool LdsLidar::LivoxLidarStart() {
  return true;
}

int LdsLidar::DeInitLdsLidar(void) {
  if (!is_initialized_) {
    printf("LiDAR data source is not exit");
    return -1;
  }

  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    for (int i = 0; i < lidar_count_; i++) {
      LidarDevice * p_lidar = &(lidars_[i]);
      if (p_lidar->lidar_type & kLivoxLidarType) {
        uint32_t handle = p_lidar->handle;
        SetLivoxLidarWorkMode(handle, kLivoxLidarWakeUp, LivoxLidarCallback::WorkModeChangeOnceCallback, nullptr);
      }
    }
    pub_handler_.Uninit();
    LivoxLidarSdkUninit();
    printf("Livox Lidar SDK Deinit completely!\n");
  }

  return 0;
}

void LdsLidar::PrepareExit(void) { DeInitLdsLidar(); }

}  // namespace livox_ros