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

#include "pub_handler.h"

#include <cstdlib>
#include <chrono>
#include <iostream>
#include <limits>

namespace livox_ros {

std::atomic<bool> PubHandler::is_timestamp_sync_;

PubHandler &pub_handler() {
  static PubHandler handler;
  return handler;
}

void PubHandler::Init() {
}

void PubHandler::Uninit() {
  if (lidar_listen_id_ > 0) {
    LivoxLidarRemovePointCloudObserver(lidar_listen_id_);
    lidar_listen_id_ = 0;
  }

  RequestExit();

  if (point_process_thread_ &&
    point_process_thread_->joinable()) {
    point_process_thread_->join();
    point_process_thread_ = nullptr;
  } else {
    /* */
  }
}

void PubHandler::RequestExit() {
  is_quit_.store(true);
}

void PubHandler::SetPointCloudConfig(const double publish_freq) {
  publish_interval_ = (kNsPerSecond / (publish_freq * 10)) * 10;
  publish_interval_tolerance_ = publish_interval_ - kNsTolerantFrameTimeDeviation;
  publish_interval_ms_ = publish_interval_ / kRatioOfMsToNs;
  if (!point_process_thread_) {
    point_process_thread_ = std::make_shared<std::thread>(&PubHandler::RawDataProcess, this);
  }
  return;
}

void PubHandler::SetImuDataCallback(ImuDataCallback cb, void* client_data) {
  imu_client_data_ = client_data;
  imu_callback_ = cb;
}

void PubHandler::SetStateInfoCallback(StateInfoCallback cb, void* client_data) {
  state_client_data_ = client_data;
  state_callback_ = cb;
  SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, this);
}

void PubHandler::LivoxLidarPushMsgCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data) {
  PubHandler* self = (PubHandler*)client_data;
  if (!self) {
    return;
  }
  if (self->state_callback_) {
      StateInfo state_info;
      state_info.lidar_type = static_cast<uint8_t>(LidarProtoType::kLivoxLidarType);
      state_info.handle = handle;

      rapidjson::Document doc;
      if (doc.Parse(info).HasParseError()) {
        return;
      }

      if (doc.HasMember("pcl_data_type")) {
        state_info.data.pcl_data_type = doc["pcl_data_type"].GetUint();
      }

      if (doc.HasMember("pattern_mode")) {
        state_info.data.pattern_mode = doc["pattern_mode"].GetUint();
      }

      // if (doc.HasMember("dual_emit_en")) {
      //   state_info.data.dual_emit_en = doc["dual_emit_en"].GetUint();
      // }

      // if (doc.HasMember("point_send_en")) {
      //   state_info.data.point_send_en = doc["point_send_en"].GetUint();
      // }

      if (doc.HasMember("lidar_ipcfg")) {
        const auto& lidar_ipcfg = doc["lidar_ipcfg"];
        state_info.data.lidar_ipcfg.ip_addr = lidar_ipcfg["lidar_ip"].GetString();
        state_info.data.lidar_ipcfg.net_mask = lidar_ipcfg["lidar_subnet_mask"].GetString();
        state_info.data.lidar_ipcfg.gw_addr = lidar_ipcfg["lidar_gateway"].GetString();
      }

      if (doc.HasMember("state_info_host_ipcfg")) {
        const auto& host_ipcfg = doc["state_info_host_ipcfg"];
        state_info.data.info_host_ipcfg.ip_addr = host_ipcfg["ip"].GetString();
        state_info.data.info_host_ipcfg.dst_port = host_ipcfg["dst_port"].GetUint();
        state_info.data.info_host_ipcfg.src_port = host_ipcfg["src_port"].GetUint();
      }

      if (doc.HasMember("ponitcloud_host_ipcfg")) {
        const auto& pointcloud_ipcfg = doc["ponitcloud_host_ipcfg"];
        state_info.data.pointcloud_host_ipcfg.ip_addr = pointcloud_ipcfg["ip"].GetString();
        state_info.data.pointcloud_host_ipcfg.dst_port = pointcloud_ipcfg["dst_port"].GetUint();
        state_info.data.pointcloud_host_ipcfg.src_port = pointcloud_ipcfg["src_port"].GetUint();
      }

      if (doc.HasMember("imu_host_ipcfg")) {
        const auto& imu_ipcfg = doc["imu_host_ipcfg"];
        state_info.data.imu_host_ipcfg.ip_addr = imu_ipcfg["ip"].GetString();
        state_info.data.imu_host_ipcfg.dst_port = imu_ipcfg["dst_port"].GetUint();
        state_info.data.imu_host_ipcfg.src_port = imu_ipcfg["src_port"].GetUint();
      }

      // if (doc.HasMember("ctl_host_ipcfg")) {
      //   const auto& ctl_ipcfg = doc["ctl_host_ipcfg"];
      //   state_info.data.ctl_host_ipcfg.ip_addr = ctl_ipcfg["ip"].GetString();
      //   state_info.data.ctl_host_ipcfg.dst_port = ctl_ipcfg["dst_port"].GetUint();
      //   state_info.data.ctl_host_ipcfg.src_port = ctl_ipcfg["src_port"].GetUint();
      // }

      // if (doc.HasMember("log_host_ipcfg")) {
      //   const auto& log_ipcfg = doc["log_host_ipcfg"];
      //   state_info.data.log_host_ipcfg.ip_addr = log_ipcfg["ip"].GetString();
      //   state_info.data.log_host_ipcfg.dst_port = log_ipcfg["dst_port"].GetUint();
      //   state_info.data.log_host_ipcfg.src_port = log_ipcfg["src_port"].GetUint();
      // }

      // if (doc.HasMember("vehicle_speed")) {
      //   state_info.data.vehicle_speed = doc["vehicle_speed"].GetInt();
      // }

      // if (doc.HasMember("environment_temp")) {
      //   state_info.data.environment_temp = doc["environment_temp"].GetInt();
      // }

      if (doc.HasMember("install_attitude")) {
        const auto& attitude = doc["install_attitude"];
        state_info.data.install_attitude.roll_deg = attitude["roll_deg"].GetDouble();
        state_info.data.install_attitude.pitch_deg = attitude["pitch_deg"].GetDouble();
        state_info.data.install_attitude.yaw_deg = attitude["yaw_deg"].GetDouble();
        state_info.data.install_attitude.x = attitude["x_mm"].GetUint();
        state_info.data.install_attitude.y = attitude["y_mm"].GetUint();
        state_info.data.install_attitude.z = attitude["z_mm"].GetUint();
      }

      // if (doc.HasMember("blind_spot_set")) {
      //   state_info.data.blind_spot_set = doc["blind_spot_set"].GetUint();
      // }

      // if (doc.HasMember("frame_rate")) {
      //   state_info.data.frame_rate = doc["frame_rate"].GetUint();
      // }

      if (doc.HasMember("fov_cfg0")) {
        const auto& fov_cfg0 = doc["fov_cfg0"];
        state_info.data.fov_cfg0.yaw_start = fov_cfg0["yaw_start"].GetInt();
        state_info.data.fov_cfg0.yaw_stop = fov_cfg0["yaw_stop"].GetInt();
        state_info.data.fov_cfg0.pitch_start = fov_cfg0["pitch_start"].GetInt();
        state_info.data.fov_cfg0.pitch_stop = fov_cfg0["pitch_stop"].GetInt();
      }

      if (doc.HasMember("fov_cfg1")) {
        const auto& fov_cfg1 = doc["fov_cfg1"];
        state_info.data.fov_cfg1.yaw_start = fov_cfg1["yaw_start"].GetInt();
        state_info.data.fov_cfg1.yaw_stop = fov_cfg1["yaw_stop"].GetInt();
        state_info.data.fov_cfg1.pitch_start = fov_cfg1["pitch_start"].GetInt();
        state_info.data.fov_cfg1.pitch_stop = fov_cfg1["pitch_stop"].GetInt();
      }

      if (doc.HasMember("fov_cfg_en")) {
        state_info.data.fov_cfg_en = doc["fov_cfg_en"].GetUint();
      }

      if (doc.HasMember("detect_mode")) {
        state_info.data.detect_mode = doc["detect_mode"].GetUint();
      }

      if (doc.HasMember("func_io_cfg")) {
        const auto& func_io_cfg = doc["func_io_cfg"];
        state_info.data.func_io_cfg[0] = func_io_cfg["IN0"].GetUint();
        state_info.data.func_io_cfg[1] = func_io_cfg["IN1"].GetUint();
        state_info.data.func_io_cfg[2] = func_io_cfg["OUT0"].GetUint();
        state_info.data.func_io_cfg[3] = func_io_cfg["OUT1"].GetUint();
      }

      if (doc.HasMember("work_tgt_mode")) {
        state_info.data.work_tgt_mode = doc["work_tgt_mode"].GetUint();
      }

      // if (doc.HasMember("glass_heat")) {
      //   state_info.data.glass_heat = doc["glass_heat"].GetUint();
      // }

      if (doc.HasMember("imu_data_en")) {
        state_info.data.imu_data_en = doc["imu_data_en"].GetUint();
      }

      // if (doc.HasMember("fusa_en")) {
      //   state_info.data.fusa_en = doc["fusa_en"].GetUint();
      // }

      if (doc.HasMember("sn")) {
        state_info.data.sn = doc["sn"].GetString();
      }

      if (doc.HasMember("product_info")) {
        state_info.data.product_info = doc["product_info"].GetString();
      }

      if (doc.HasMember("version_app")) {
        const auto& version_app = doc["version_app"];
        for (rapidjson::SizeType i = 0; i < version_app.Size(); ++i) {
          state_info.data.version_app[i] = version_app[i].GetUint();
        }
      }

      if (doc.HasMember("version_loader")) {
        const auto& version_loader = doc["version_loader"];
        for (rapidjson::SizeType i = 0; i < version_loader.Size(); ++i) {
          state_info.data.version_loader[i] = version_loader[i].GetUint();
        }
      }

      if (doc.HasMember("version_hardware")) {
        const auto& version_hardware = doc["version_hardware"];
        for (rapidjson::SizeType i = 0; i < version_hardware.Size(); ++i) {
          state_info.data.version_hardware[i] = version_hardware[i].GetUint();
        }
      }

      if (doc.HasMember("mac")) {
        const auto& mac = doc["mac"];
        for (rapidjson::SizeType i = 0; i < mac.Size(); ++i) {
          state_info.data.mac[i] = mac[i].GetUint();
        }
      }

      if (doc.HasMember("cur_work_state")) {
        state_info.data.cur_work_state = doc["cur_work_state"].GetUint();
      }

      if (doc.HasMember("core_temp")) {
        state_info.data.core_temp = doc["core_temp"].GetInt();
      }

      if (doc.HasMember("powerup_cnt")) {
        state_info.data.powerup_cnt = doc["powerup_cnt"].GetUint();
      }

      if (doc.HasMember("local_time_now")) {
        state_info.data.local_time_now = doc["local_time_now"].GetUint64();
      }

      if (doc.HasMember("last_sync_time")) {
        state_info.data.last_sync_time = doc["last_sync_time"].GetUint64();
      }

      if (doc.HasMember("time_offset")) {
        state_info.data.time_offset = doc["time_offset"].GetInt64();
      }

      if (doc.HasMember("time_sync_type")) {
        state_info.data.time_sync_type = doc["time_sync_type"].GetUint();
      }

      // if (doc.HasMember("status_code")) {
      //   std::istringstream ss(doc["status_code"].GetString());
      //   for (int idx = 31; idx >= 0; --idx) {
      //     ss >> std::hex >> state_info.data.status_code[idx];
      //   }
      // }

      if (doc.HasMember("lidar_diag_status")) {
        state_info.data.lidar_diag_status = doc["lidar_diag_status"].GetUint();
      }

      // if (doc.HasMember("lidar_flash_status")) {
      //   state_info.data.lidar_flash_status = doc["lidar_flash_status"].GetUint();
      // }

      if (doc.HasMember("FW_TYPE")) {
        state_info.data.fw_type = doc["FW_TYPE"].GetUint();
      }

      if (doc.HasMember("hms_code")) {
        const auto& hms_code = doc["hms_code"];
        for (rapidjson::SizeType i = 0; i < hms_code.Size(); ++i) {
          state_info.data.hms_code[i] = hms_code[i].GetUint();
        }
      }

      // if (doc.HasMember("ROI_Mode")) {
      //   state_info.data.ROI_Mode = doc["ROI_Mode"].GetUint();
      // }

      self->state_callback_(&state_info, self->state_client_data_);
    }

  return;
}

void PubHandler::AddLidarsExtParam(LidarExtParameter& lidar_param) {
  std::unique_lock<std::mutex> lock(packet_mutex_);
  uint32_t id = 0;
  GetLidarId(lidar_param.lidar_type, lidar_param.handle, id);
  lidar_extrinsics_[id] = lidar_param;
}

void PubHandler::ClearAllLidarsExtrinsicParams() {
  std::unique_lock<std::mutex> lock(packet_mutex_);
  lidar_extrinsics_.clear();
}

void PubHandler::SetPointCloudsCallback(PointCloudsCallback cb, void* client_data) {
  pub_client_data_ = client_data;
  points_callback_ = cb;
  lidar_listen_id_ = LivoxLidarAddPointCloudObserver(OnLivoxLidarPointCloudCallback, this);
}

void PubHandler::OnLivoxLidarPointCloudCallback(uint32_t handle, const uint8_t dev_type,
                                                LivoxLidarEthernetPacket *data, void *client_data) {
  PubHandler* self = (PubHandler*)client_data;
  if (!self) {
    return;
  }

  if (data->time_type != kTimestampTypeNoSync) {
    is_timestamp_sync_.store(true);
  } else {
    is_timestamp_sync_.store(false);
  }

  if (data->data_type == kLivoxLidarImuData) {
    if (self->imu_callback_) {
      RawImuPoint* imu = (RawImuPoint*) data->data;
      ImuData imu_data;
      imu_data.lidar_type = static_cast<uint8_t>(LidarProtoType::kLivoxLidarType);
      imu_data.handle = handle;
      imu_data.time_stamp = GetEthPacketTimestamp(data->time_type,
                                                  data->timestamp, sizeof(data->timestamp));
      imu_data.gyro_x = imu->gyro_x;
      imu_data.gyro_y = imu->gyro_y;
      imu_data.gyro_z = imu->gyro_z;
      imu_data.acc_x = imu->acc_x;
      imu_data.acc_y = imu->acc_y;
      imu_data.acc_z = imu->acc_z;
      self->imu_callback_(&imu_data, self->imu_client_data_);
    }
    return;
  }
  RawPacket packet = {};
  packet.handle = handle;
  packet.lidar_type = LidarProtoType::kLivoxLidarType;
  packet.extrinsic_enable = false; 
  if (dev_type == LivoxLidarDeviceType::kLivoxLidarTypeIndustrialHAP) {
    packet.line_num = kLineNumberHAP;
  } else if (dev_type == LivoxLidarDeviceType::kLivoxLidarTypeMid360) {
    packet.line_num = kLineNumberMid360;
  } else {
    packet.line_num = kLineNumberDefault;
  }
  packet.data_type = data->data_type;
  packet.point_num = data->dot_num;
  packet.point_interval = data->time_interval * 100 / data->dot_num;  //ns
  packet.time_stamp = GetEthPacketTimestamp(data->time_type,
                                            data->timestamp, sizeof(data->timestamp));
  uint32_t length = data->length - sizeof(LivoxLidarEthernetPacket) + 1;
  packet.raw_data.insert(packet.raw_data.end(), data->data, data->data + length);
  {
    std::unique_lock<std::mutex> lock(self->packet_mutex_);
    self->raw_packet_queue_.push_back(packet);
  }
    self->packet_condition_.notify_one();

  return;
}

void PubHandler::PublishPointCloud() {
  //publish point
  if (points_callback_) {
    points_callback_(&frame_, pub_client_data_);
  }
  return;
}

void PubHandler::CheckTimer(uint32_t id) {

  if (PubHandler::is_timestamp_sync_.load()) { // Enable time synchronization
    auto& process_handler = lidar_process_handlers_[id];
    uint64_t recent_time_ms = process_handler->GetRecentTimeStamp() / kRatioOfMsToNs;
    if ((recent_time_ms % publish_interval_ms_ != 0) || recent_time_ms == 0) {
      return;
    }

    uint64_t diff = process_handler->GetRecentTimeStamp() - process_handler->GetLidarBaseTime();
    if (diff < publish_interval_tolerance_) {
      return;
    }

    frame_.base_time[frame_.lidar_num] = process_handler->GetLidarBaseTime();
    points_[id].clear();
    process_handler->GetLidarPointClouds(points_[id]);
    if (points_[id].empty()) {
      return;
    }
    PointPacket& lidar_point = frame_.lidar_point[frame_.lidar_num];
    lidar_point.lidar_type = LidarProtoType::kLivoxLidarType;  // TODO:
    lidar_point.handle = id;
    lidar_point.points_num = points_[id].size();
    lidar_point.points = points_[id].data();
    frame_.lidar_num++;
    
    if (frame_.lidar_num != 0) {
      PublishPointCloud();
      frame_.lidar_num = 0;
    }
  } else { // Disable time synchronization
    auto now_time = std::chrono::high_resolution_clock::now();
    //First Set
    static bool first = true;
    if (first) {
      last_pub_time_ = now_time;
      first = false;
      return;
    }
    if (now_time - last_pub_time_ < std::chrono::nanoseconds(publish_interval_)) {
      return;
    }
    last_pub_time_ += std::chrono::nanoseconds(publish_interval_);
    for (auto &process_handler : lidar_process_handlers_) {
      frame_.base_time[frame_.lidar_num] = process_handler.second->GetLidarBaseTime();
      uint32_t handle = process_handler.first;
      points_[handle].clear();
      process_handler.second->GetLidarPointClouds(points_[handle]);
      if (points_[handle].empty()) {
        continue;
      }
      PointPacket& lidar_point = frame_.lidar_point[frame_.lidar_num];
      lidar_point.lidar_type = LidarProtoType::kLivoxLidarType;  // TODO:
      lidar_point.handle = handle;
      lidar_point.points_num = points_[handle].size();
      lidar_point.points = points_[handle].data();
      frame_.lidar_num++;
    }
    PublishPointCloud();
    frame_.lidar_num = 0;
  }
  return;
}

void PubHandler::RawDataProcess() {
  RawPacket raw_data;
  while (!is_quit_.load()) {
    {
      std::unique_lock<std::mutex> lock(packet_mutex_);
      if (raw_packet_queue_.empty()) {
        packet_condition_.wait_for(lock, std::chrono::milliseconds(500));
        if (raw_packet_queue_.empty()) {
          continue;
        }
      }
      raw_data = raw_packet_queue_.front();
      raw_packet_queue_.pop_front();
    }
    uint32_t id = 0;
    GetLidarId(raw_data.lidar_type, raw_data.handle, id);
    if (lidar_process_handlers_.find(id) == lidar_process_handlers_.end()) {
      lidar_process_handlers_[id].reset(new LidarPubHandler());
    }
    auto &process_handler = lidar_process_handlers_[id];
    if (lidar_extrinsics_.find(id) != lidar_extrinsics_.end()) {
        lidar_process_handlers_[id]->SetLidarsExtParam(lidar_extrinsics_[id]);
    }
    process_handler->PointCloudProcess(raw_data);
    CheckTimer(id);
  }
}

bool PubHandler::GetLidarId(LidarProtoType lidar_type, uint32_t handle, uint32_t& id) {
  if (lidar_type == kLivoxLidarType) {
    id = handle;
    return true;
  }
  return false;
}

uint64_t PubHandler::GetEthPacketTimestamp(uint8_t timestamp_type, uint8_t* time_stamp, uint8_t size) {
  LdsStamp time;
  memcpy(time.stamp_bytes, time_stamp, size);

  if (timestamp_type == kTimestampTypeGptpOrPtp ||
      timestamp_type == kTimestampTypeGps) {
    return time.stamp;
  }

  return std::chrono::high_resolution_clock::now().time_since_epoch().count();
}

/*******************************/
/*  LidarPubHandler Definitions*/
LidarPubHandler::LidarPubHandler() : is_set_extrinsic_params_(false) {}

uint64_t LidarPubHandler::GetLidarBaseTime() {
  if (points_clouds_.empty()) {
    return 0;
  }
  return points_clouds_.at(0).offset_time;
}

void LidarPubHandler::GetLidarPointClouds(std::vector<PointXyzlt>& points_clouds) {
  std::lock_guard<std::mutex> lock(mutex_);
  points_clouds.swap(points_clouds_);
}

uint64_t LidarPubHandler::GetRecentTimeStamp() {
  if (points_clouds_.empty()) {
    return 0;
  }
  return points_clouds_.back().offset_time;
}

uint32_t LidarPubHandler::GetLidarPointCloudsSize() {
  std::lock_guard<std::mutex> lock(mutex_);
  return points_clouds_.size();
}

//convert to standard format and extrinsic compensate
void LidarPubHandler::PointCloudProcess(RawPacket & pkt) {
  if (pkt.lidar_type == LidarProtoType::kLivoxLidarType) {
    LivoxLidarPointCloudProcess(pkt);
  } else {
    static bool flag = false;
    if (!flag) {
      std::cout << "error, unsupported protocol type: " << static_cast<int>(pkt.lidar_type) << std::endl;
      flag = true;      
    }
  }
}

void LidarPubHandler::LivoxLidarPointCloudProcess(RawPacket & pkt) {
  switch (pkt.data_type) {
    case kLivoxLidarCartesianCoordinateHighData:
      ProcessCartesianHighPoint(pkt);
      break;
    case kLivoxLidarCartesianCoordinateLowData:
      ProcessCartesianLowPoint(pkt);
      break;
    case kLivoxLidarSphericalCoordinateData:
      ProcessSphericalPoint(pkt);
      break;
    default:
      std::cout << "unknown data type: " << static_cast<int>(pkt.data_type)
                << " !!" << std::endl;
      break;
  }
}

void LidarPubHandler::SetLidarsExtParam(LidarExtParameter lidar_param) {
  if (is_set_extrinsic_params_) {
    return;
  }
  extrinsic_.trans[0] = lidar_param.param.x;
  extrinsic_.trans[1] = lidar_param.param.y;
  extrinsic_.trans[2] = lidar_param.param.z;

  double cos_roll = cos(static_cast<double>(lidar_param.param.roll * PI / 180.0));
  double cos_pitch = cos(static_cast<double>(lidar_param.param.pitch * PI / 180.0));
  double cos_yaw = cos(static_cast<double>(lidar_param.param.yaw * PI / 180.0));
  double sin_roll = sin(static_cast<double>(lidar_param.param.roll * PI / 180.0));
  double sin_pitch = sin(static_cast<double>(lidar_param.param.pitch * PI / 180.0));
  double sin_yaw = sin(static_cast<double>(lidar_param.param.yaw * PI / 180.0));

  extrinsic_.rotation[0][0] = cos_pitch * cos_yaw;
  extrinsic_.rotation[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
  extrinsic_.rotation[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;

  extrinsic_.rotation[1][0] = cos_pitch * sin_yaw;
  extrinsic_.rotation[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
  extrinsic_.rotation[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;

  extrinsic_.rotation[2][0] = -sin_pitch;
  extrinsic_.rotation[2][1] = sin_roll * cos_pitch;
  extrinsic_.rotation[2][2] = cos_roll * cos_pitch;

  is_set_extrinsic_params_ = true;
}

void LidarPubHandler::ProcessCartesianHighPoint(RawPacket & pkt) {
  LivoxLidarCartesianHighRawPoint* raw = (LivoxLidarCartesianHighRawPoint*)pkt.raw_data.data();
  PointXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      point.x = raw[i].x / 1000.0;
      point.y = raw[i].y / 1000.0;
      point.z = raw[i].z / 1000.0;
    } else {
      point.x = (raw[i].x * extrinsic_.rotation[0][0] +
                raw[i].y * extrinsic_.rotation[0][1] +
                raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 1000.0;
      point.y = (raw[i].x* extrinsic_.rotation[1][0] +
                raw[i].y * extrinsic_.rotation[1][1] +
                raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 1000.0;
      point.z = (raw[i].x * extrinsic_.rotation[2][0] +
                raw[i].y * extrinsic_.rotation[2][1] +
                raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 1000.0;
    }
    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::ProcessCartesianLowPoint(RawPacket & pkt) {
  LivoxLidarCartesianLowRawPoint* raw = (LivoxLidarCartesianLowRawPoint*)pkt.raw_data.data();
  PointXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    if (pkt.extrinsic_enable) {
      point.x = raw[i].x / 100.0;
      point.y = raw[i].y / 100.0;
      point.z = raw[i].z / 100.0;
    } else {
      point.x = (raw[i].x * extrinsic_.rotation[0][0] +
                raw[i].y * extrinsic_.rotation[0][1] +
                raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 100.0;
      point.y = (raw[i].x* extrinsic_.rotation[1][0] +
                raw[i].y * extrinsic_.rotation[1][1] +
                raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 100.0;
      point.z = (raw[i].x * extrinsic_.rotation[2][0] +
                raw[i].y * extrinsic_.rotation[2][1] +
                raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 100.0;
    }
    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

void LidarPubHandler::ProcessSphericalPoint(RawPacket& pkt) {
  LivoxLidarSpherPoint* raw = (LivoxLidarSpherPoint*)pkt.raw_data.data();
  PointXyzlt point = {};
  for (uint32_t i = 0; i < pkt.point_num; i++) {
    double radius = raw[i].depth / 1000.0;
    double theta = raw[i].theta / 100.0 / 180 * PI;
    double phi = raw[i].phi / 100.0 / 180 * PI;
    double src_x = radius * sin(theta) * cos(phi);
    double src_y = radius * sin(theta) * sin(phi);
    double src_z = radius * cos(theta);
    if (pkt.extrinsic_enable) {
      point.x = src_x;
      point.y = src_y;
      point.z = src_z;
    } else {
      point.x = src_x * extrinsic_.rotation[0][0] +
                src_y * extrinsic_.rotation[0][1] +
                src_z * extrinsic_.rotation[0][2] + (extrinsic_.trans[0] / 1000.0);
      point.y = src_x * extrinsic_.rotation[1][0] +
                src_y * extrinsic_.rotation[1][1] +
                src_z * extrinsic_.rotation[1][2] + (extrinsic_.trans[1] / 1000.0);
      point.z = src_x * extrinsic_.rotation[2][0] +
                src_y * extrinsic_.rotation[2][1] +
                src_z * extrinsic_.rotation[2][2] + (extrinsic_.trans[2] / 1000.0);
    }

    point.intensity = raw[i].reflectivity;
    point.line = i % pkt.line_num;
    point.tag = raw[i].tag;
    point.offset_time = pkt.time_stamp + i * pkt.point_interval;
    std::lock_guard<std::mutex> lock(mutex_);
    points_clouds_.push_back(point);
  }
}

} // namespace livox_ros
