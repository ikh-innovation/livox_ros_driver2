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

#include "lddc.h"
#include "comm/ldq.h"
#include "comm/comm.h"

#include <inttypes.h>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <stdint.h>

#include "include/ros_headers.h"

#include "driver_node.h"
#include "lds_lidar.h"

namespace livox_ros {

/** Lidar Data Distribute Control--------------------------------------------*/
Lddc::Lddc(int format, int multi_topic, int data_src, int output_type,
    double frq, std::string &frame_id, bool lidar_bag, bool imu_bag)
    : transfer_format_(format),
      use_multi_topic_(multi_topic),
      data_src_(data_src),
      output_type_(output_type),
      publish_frq_(frq),
      frame_id_(frame_id),
      enable_lidar_bag_(lidar_bag),
      enable_imu_bag_(imu_bag) {
  publish_period_ns_ = kNsPerSecond / publish_frq_;
  lds_ = nullptr;
  memset(private_pub_, 0, sizeof(private_pub_));
  memset(private_imu_pub_, 0, sizeof(private_imu_pub_));
  memset(private_state_info_pub_, 0, sizeof(private_state_info_pub_));
  global_pub_ = nullptr;
  global_imu_pub_ = nullptr;
  global_state_info_pub_ = nullptr;
  cur_node_ = nullptr;
  bag_ = nullptr;
}

Lddc::~Lddc() {
  if (global_pub_) {
    delete global_pub_;
  }

  if (global_imu_pub_) {
    delete global_imu_pub_;
  }

  if (global_state_info_pub_) {
    delete global_state_info_pub_;
  }

  PrepareExit();

  for (uint32_t i = 0; i < kMaxSourceLidar; i++) {
    if (private_pub_[i]) {
      delete private_pub_[i];
    }
  }

  for (uint32_t i = 0; i < kMaxSourceLidar; i++) {
    if (private_imu_pub_[i]) {
      delete private_imu_pub_[i];
    }
  }

  for (uint32_t i = 0; i < kMaxSourceLidar; i++) {
    if (private_state_info_pub_[i]) {
      delete private_state_info_pub_[i];
    }
  }
  std::cout << "lddc destory!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
}

int Lddc::RegisterLds(Lds *lds) {
  if (lds_ == nullptr) {
    lds_ = lds;
    return 0;
  } else {
    return -1;
  }
}

void Lddc::DistributePointCloudData(void) {
  if (!lds_) {
    std::cout << "lds is not registered" << std::endl;
    return;
  }
  if (lds_->IsRequestExit()) {
    std::cout << "DistributePointCloudData is RequestExit" << std::endl;
    return;
  }
  
  lds_->pcd_semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    LidarDataQueue *p_queue = &lidar->data;
    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
      continue;
    }
    PollingLidarPointCloudData(lidar_id, lidar);    
  }
}

void Lddc::DistributeImuData(void) {
  if (!lds_) {
    std::cout << "lds is not registered" << std::endl;
    return;
  }
  if (lds_->IsRequestExit()) {
    std::cout << "DistributeImuData is RequestExit" << std::endl;
    return;
  }
  
  lds_->imu_semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    LidarImuDataQueue *p_queue = &lidar->imu_data;
    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
      continue;
    }
    PollingLidarImuData(lidar_id, lidar);
  }
}

void Lddc::DistributeStateInfo(void) {
  if (!lds_) {
    std::cout << "lds is not registered" << std::endl;
    return;
  }
  if (lds_->IsRequestExit()) {
    std::cout << "DistributeStateInfo is RequestExit" << std::endl;
    return;
  }
  
  lds_->state_info_semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    LidarStateInfoQueue *p_queue = &lidar->state_info;
    if (p_queue == nullptr) {
      continue;
    }
    PollingLidarStateInfo(lidar_id, lidar);
  }
}

void Lddc::PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar) {
  LidarDataQueue *p_queue = &lidar->data;
  if (p_queue == nullptr || p_queue->storage_packet == nullptr) {
    return;
  }

  while (!lds_->IsRequestExit() && !QueueIsEmpty(p_queue)) {
    if (kPointCloud2Msg == transfer_format_) {
      PublishPointcloud2(p_queue, index);
    } else if (kLivoxCustomMsg == transfer_format_) {
      PublishCustomPointcloud(p_queue, index);
    } else if (kPclPxyziMsg == transfer_format_) {
      PublishPclMsg(p_queue, index);
    }
  }
}

void Lddc::PollingLidarImuData(uint8_t index, LidarDevice *lidar) {
  LidarImuDataQueue& p_queue = lidar->imu_data;
  while (!lds_->IsRequestExit() && !p_queue.Empty()) {
    PublishImuData(p_queue, index);
  }
}

void Lddc::PollingLidarStateInfo(uint8_t index, LidarDevice *lidar) {
  LidarStateInfoQueue& p_queue = lidar->state_info;
  while (!lds_->IsRequestExit() && !p_queue.Empty()) {
    PublishStateInfo(p_queue, index);
  }
}

void Lddc::PrepareExit(void) {
  if (bag_) {
    DRIVER_INFO(*cur_node_, "Waiting to save the bag file!");
    bag_->close();
    DRIVER_INFO(*cur_node_, "Save the bag file successfully!");
    bag_ = nullptr;
  }
  
  if (lds_) {
    lds_->PrepareExit();
    lds_ = nullptr;
  }
}

std::vector<std::string> Lddc::GetStateTopics(void)
{
  std::vector<std::string> topics{};

  if (use_multi_topic_) 
  {
    for (uint32_t i = 0; i < lds_->lidar_count_; i++) 
    {
      if (private_state_info_pub_[i] != nullptr)
      {
        topics.push_back(private_state_info_pub_[i]->getTopic());
      }
    }
  }   
  else 
  {
    if (global_state_info_pub_ != nullptr)
    {
      topics.push_back(global_state_info_pub_->getTopic());
    }    
  }

  return topics;
}

void Lddc::PublishPointcloud2(LidarDataQueue *queue, uint8_t index) {
  while(!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) {
      printf("Publish point cloud2 failed, the pkg points is empty.\n");
      continue;
    }

    PointCloud2Ptr cloud(new PointCloud2);
    uint64_t timestamp = 0;
    InitPointcloud2Msg(pkg, cloud, timestamp);
    PublishPointcloud2Data(index, timestamp, cloud);
  }
}

void Lddc::PublishCustomPointcloud(LidarDataQueue *queue, uint8_t index) {
  while(!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) {
      printf("Publish custom point cloud failed, the pkg points is empty.\n");
      continue;
    }

    CustomMsgPtr livox_msg(new CustomMsg);
    InitCustomMsg(livox_msg, pkg, index);
    FillPointsToCustomMsg(livox_msg, pkg);
    PublishCustomPointData(livox_msg, index);
  }
}

/* for pcl::pxyzi */
void Lddc::PublishPclMsg(LidarDataQueue *queue, uint8_t index) {
  while(!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) {
      printf("Publish point cloud failed, the pkg points is empty.\n");
      continue;
    }

    PointCloud cloud;
    uint64_t timestamp = 0;
    InitPclMsg(pkg, cloud, timestamp);
    FillPointsToPclMsg(pkg, cloud);
    PublishPclData(index, timestamp, cloud);
  }
  return;
}

void Lddc::InitPointcloud2MsgHeader(PointCloud2Ptr& cloud) {
  cloud->header.frame_id.assign(frame_id_);
  cloud->height = 1;
  cloud->width = 0;
  cloud->fields.resize(7);
  cloud->fields[0].offset = 0;
  cloud->fields[0].name = "x";
  cloud->fields[0].count = 1;
  cloud->fields[0].datatype = PointField::FLOAT32;
  cloud->fields[1].offset = 4;
  cloud->fields[1].name = "y";
  cloud->fields[1].count = 1;
  cloud->fields[1].datatype = PointField::FLOAT32;
  cloud->fields[2].offset = 8;
  cloud->fields[2].name = "z";
  cloud->fields[2].count = 1;
  cloud->fields[2].datatype = PointField::FLOAT32;
  cloud->fields[3].offset = 12;
  cloud->fields[3].name = "intensity";
  cloud->fields[3].count = 1;
  cloud->fields[3].datatype = PointField::FLOAT32;
  cloud->fields[4].offset = 16;
  cloud->fields[4].name = "tag";
  cloud->fields[4].count = 1;
  cloud->fields[4].datatype = PointField::UINT8;
  cloud->fields[5].offset = 17;
  cloud->fields[5].name = "line";
  cloud->fields[5].count = 1;
  cloud->fields[5].datatype = PointField::UINT8;
  cloud->fields[6].offset = 18;
  cloud->fields[6].name = "timestamp";
  cloud->fields[6].count = 1;
  cloud->fields[6].datatype = PointField::UINT32;
  cloud->point_step = sizeof(LivoxPointXyzrtlt);
}

void Lddc::InitPointcloud2Msg(const StoragePacket& pkg, PointCloud2Ptr& cloud, uint64_t& timestamp) {
  InitPointcloud2MsgHeader(cloud);

  cloud->point_step = sizeof(LivoxPointXyzrtlt);

  cloud->width = pkg.points_num;
  cloud->row_step = cloud->width * cloud->point_step;

  cloud->is_bigendian = false;
  cloud->is_dense     = true;

  if (!pkg.points.empty()) {
    timestamp = pkg.base_time;
  }

  cloud->header.stamp = ros::Time((int32_t)(timestamp / 1000000000), (int32_t)(timestamp % 1000000000));

  std::vector<LivoxPointXyzrtlt> points;
  for (size_t i = 0; i < pkg.points_num; ++i) {
    LivoxPointXyzrtlt point;
    point.x = pkg.points[i].x;
    point.y = pkg.points[i].y;
    point.z = pkg.points[i].z;
    point.reflectivity = pkg.points[i].intensity;
    point.tag = pkg.points[i].tag;
    point.line = pkg.points[i].line;
    point.timestamp = static_cast<uint32_t>(pkg.points[i].offset_time - pkg.base_time);
    points.push_back(std::move(point));
  }
  cloud->data.resize(pkg.points_num * sizeof(LivoxPointXyzrtlt));
  memcpy(cloud->data.data(), points.data(), pkg.points_num * sizeof(LivoxPointXyzrtlt));
}

void Lddc::PublishPointcloud2Data(const uint8_t index, const uint64_t timestamp, const PointCloud2Ptr& cloud) {
PublisherPtr publisher_ptr = Lddc::GetCurrentPublisher(index);

  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(cloud);
  } else {
    if (bag_ && enable_lidar_bag_) {
      bag_->write(publisher_ptr->getTopic(), ros::Time((int32_t)(timestamp / 1000000000), (int32_t)(timestamp % 1000000000)), cloud);
    }
  }
}

void Lddc::InitCustomMsg(CustomMsgPtr& livox_msg, const StoragePacket& pkg, uint8_t index) {
  livox_msg->header.frame_id.assign(frame_id_);

  static uint32_t msg_seq = 0;
  livox_msg->header.seq = msg_seq;
  ++msg_seq;

  uint64_t timestamp = 0;
  if (!pkg.points.empty()) {
    timestamp = pkg.base_time;
  }
  livox_msg->timebase = timestamp;
  livox_msg->header.stamp = ros::Time((int32_t)(timestamp / 1000000000), (int32_t)(timestamp % 1000000000));
  livox_msg->point_num = pkg.points_num;
  if (lds_->lidars_[index].lidar_type == kLivoxLidarType) {
    livox_msg->lidar_id = lds_->lidars_[index].handle;
  } else {
    printf("Init custom msg lidar id failed, the index:%u.\n", index);
    livox_msg->lidar_id = 0;
  }
}

void Lddc::FillPointsToCustomMsg(CustomMsgPtr& livox_msg, const StoragePacket& pkg) {
  uint32_t points_num = pkg.points_num;
  const std::vector<PointXyzlt>& points = pkg.points;
  for (uint32_t i = 0; i < points_num; ++i) {
    CustomPoint point;
    point.x = points[i].x;
    point.y = points[i].y;
    point.z = points[i].z;
    point.reflectivity = points[i].intensity;
    point.tag = points[i].tag;
    point.line = points[i].line;
    point.offset_time = static_cast<uint32_t>(points[i].offset_time - pkg.base_time);

    livox_msg->points.push_back(std::move(point));
  }
}

void Lddc::PublishCustomPointData(const CustomMsgPtr& livox_msg, const uint8_t index) {
  PublisherPtr publisher_ptr = Lddc::GetCurrentPublisher(index);

  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(livox_msg);
  } else {
    if (bag_ && enable_lidar_bag_) {
      bag_->write(publisher_ptr->getTopic(), ros::Time((int32_t)(livox_msg->timebase / 1000000000), (int32_t)(livox_msg->timebase % 1000000000)), livox_msg);
    }
  }
}

void Lddc::InitPclMsg(const StoragePacket& pkg, PointCloud& cloud, uint64_t& timestamp) {
  cloud.header.frame_id.assign(frame_id_);
  cloud.height = 1;
  cloud.width = pkg.points_num;

  if (!pkg.points.empty()) {
    timestamp = pkg.base_time;
  }
  cloud.header.stamp = timestamp / 1000.0;  // to pcl ros time stamp
  return;
}

void Lddc::FillPointsToPclMsg(const StoragePacket& pkg, PointCloud& pcl_msg) {
  if (pkg.points.empty()) {
    return;
  }

  uint32_t points_num = pkg.points_num;
  const std::vector<PointXyzlt>& points = pkg.points;
  for (uint32_t i = 0; i < points_num; ++i) {
    pcl::PointXYZI point;
    point.x = points[i].x;
    point.y = points[i].y;
    point.z = points[i].z;
    point.intensity = points[i].intensity;

    pcl_msg.points.push_back(std::move(point));
  }
  return;
}

void Lddc::PublishPclData(const uint8_t index, const uint64_t timestamp, const PointCloud& cloud) {
  PublisherPtr publisher_ptr = Lddc::GetCurrentPublisher(index);
  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(cloud);
  } else {
    if (bag_ && enable_lidar_bag_) {
      bag_->write(publisher_ptr->getTopic(), ros::Time((int32_t)(timestamp / 1000000000), (int32_t)(timestamp % 1000000000)), cloud);
    }
  }
  return;
}

void Lddc::InitImuMsg(const ImuData& imu_data, ImuMsg& imu_msg, uint64_t& timestamp) {
  imu_msg.header.frame_id = "livox_frame";

  timestamp = imu_data.time_stamp;
  imu_msg.header.stamp = ros::Time((int32_t)(timestamp / 1000000000), (int32_t)(timestamp % 1000000000));  // to ros time stamp
  imu_msg.angular_velocity.x = imu_data.gyro_x;
  imu_msg.angular_velocity.y = imu_data.gyro_y;
  imu_msg.angular_velocity.z = imu_data.gyro_z;
  imu_msg.linear_acceleration.x = imu_data.acc_x;
  imu_msg.linear_acceleration.y = imu_data.acc_y;
  imu_msg.linear_acceleration.z = imu_data.acc_z;
}

void Lddc::PublishImuData(LidarImuDataQueue& imu_data_queue, const uint8_t index) {
  ImuData imu_data;
  if (!imu_data_queue.Pop(imu_data)) {
    //printf("Publish imu data failed, imu data queue pop failed.\n");
    return;
  }

  ImuMsg imu_msg;
  uint64_t timestamp;
  InitImuMsg(imu_data, imu_msg, timestamp);

  PublisherPtr publisher_ptr = GetCurrentImuPublisher(index);

  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(imu_msg);
  } else {
    if (bag_ && enable_imu_bag_) {
      bag_->write(publisher_ptr->getTopic(), ros::Time((int32_t)(timestamp / 1000000000), (int32_t)(timestamp % 1000000000)), imu_msg);
    }
  }
}

void Lddc::PublishStateInfo(LidarStateInfoQueue& state_info_queue, const uint8_t index) {
  StateInfo state_info;
  if (!state_info_queue.Pop(state_info)) {
    //printf("Publish state info failed, state info queue pop failed.\n");
    return;
  }

  StateInfoMsg state_info_msg;

  state_info_msg.pcl_data_type = state_info.data.pcl_data_type;
  state_info_msg.pattern_mode = state_info.data.pattern_mode;
  // state_info_msg.dual_emit_en = state_info.data.dual_emit_en;
  // state_info_msg.point_send_en = state_info.data.point_send_en;

  state_info_msg.lidar_ipcfg.ip_addr = state_info.data.lidar_ipcfg.ip_addr;
  state_info_msg.lidar_ipcfg.net_mask = state_info.data.lidar_ipcfg.net_mask;
  state_info_msg.lidar_ipcfg.gw_addr = state_info.data.lidar_ipcfg.gw_addr;

  state_info_msg.info_host_ipcfg.ip_addr = state_info.data.info_host_ipcfg.ip_addr;
  state_info_msg.info_host_ipcfg.dst_port = state_info.data.info_host_ipcfg.dst_port;
  state_info_msg.info_host_ipcfg.src_port = state_info.data.info_host_ipcfg.src_port;

  state_info_msg.pointcloud_host_ipcfg.ip_addr = state_info.data.pointcloud_host_ipcfg.ip_addr;
  state_info_msg.pointcloud_host_ipcfg.dst_port = state_info.data.pointcloud_host_ipcfg.dst_port;
  state_info_msg.pointcloud_host_ipcfg.src_port = state_info.data.pointcloud_host_ipcfg.src_port;

  state_info_msg.imu_host_ipcfg.ip_addr = state_info.data.imu_host_ipcfg.ip_addr;
  state_info_msg.imu_host_ipcfg.dst_port = state_info.data.imu_host_ipcfg.dst_port;
  state_info_msg.imu_host_ipcfg.src_port = state_info.data.imu_host_ipcfg.src_port;

  // state_info_msg.ctl_host_ipcfg.ip_addr = state_info.data.ctl_host_ipcfg.ip_addr;
  // state_info_msg.ctl_host_ipcfg.dst_port = state_info.data.ctl_host_ipcfg.dst_port;
  // state_info_msg.ctl_host_ipcfg.src_port = state_info.data.ctl_host_ipcfg.src_port;

  // state_info_msg.log_host_ipcfg.ip_addr = state_info.data.log_host_ipcfg.ip_addr;
  // state_info_msg.log_host_ipcfg.dst_port = state_info.data.log_host_ipcfg.dst_port;
  // state_info_msg.log_host_ipcfg.src_port = state_info.data.log_host_ipcfg.src_port;

  // state_info_msg.vehicle_speed = state_info.data.vehicle_speed;
  // state_info_msg.environment_temp = state_info.data.environment_temp;

  state_info_msg.install_attitude.roll_deg = state_info.data.install_attitude.roll_deg;
  state_info_msg.install_attitude.pitch_deg = state_info.data.install_attitude.pitch_deg;
  state_info_msg.install_attitude.yaw_deg = state_info.data.install_attitude.yaw_deg;
  state_info_msg.install_attitude.x = state_info.data.install_attitude.x;
  state_info_msg.install_attitude.y = state_info.data.install_attitude.y;
  state_info_msg.install_attitude.z = state_info.data.install_attitude.z;

  // state_info_msg.blind_spot_set = state_info.data.blind_spot_set;
  // state_info_msg.frame_rate = state_info.data.frame_rate;

  state_info_msg.fov_cfg0.yaw_start = state_info.data.fov_cfg0.yaw_start;
  state_info_msg.fov_cfg0.yaw_stop = state_info.data.fov_cfg0.yaw_stop;
  state_info_msg.fov_cfg0.pitch_start = state_info.data.fov_cfg0.pitch_start;
  state_info_msg.fov_cfg0.pitch_stop = state_info.data.fov_cfg0.pitch_stop;

  state_info_msg.fov_cfg1.yaw_start = state_info.data.fov_cfg1.yaw_start;
  state_info_msg.fov_cfg1.yaw_stop = state_info.data.fov_cfg1.yaw_stop;
  state_info_msg.fov_cfg1.pitch_start = state_info.data.fov_cfg1.pitch_start;
  state_info_msg.fov_cfg1.pitch_stop = state_info.data.fov_cfg1.pitch_stop;

  state_info_msg.fov_cfg_en = state_info.data.fov_cfg_en;
  state_info_msg.detect_mode = state_info.data.detect_mode;

  for (size_t i = 0; i < 4; ++i) {
    state_info_msg.func_io_cfg[i] = state_info.data.func_io_cfg[i];
  }

  state_info_msg.work_tgt_mode = state_info.data.work_tgt_mode;
  // state_info_msg.glass_heat = state_info.data.glass_heat;
  state_info_msg.imu_data_en = state_info.data.imu_data_en;
  // state_info_msg.fusa_en = state_info.data.fusa_en;
  state_info_msg.sn = state_info.data.sn;
  state_info_msg.product_info = state_info.data.product_info;

  for (size_t i = 0; i < 4; ++i) {
    state_info_msg.version_app[i] = state_info.data.version_app[i];
    state_info_msg.version_loader[i] = state_info.data.version_loader[i];
    state_info_msg.version_hardware[i] = state_info.data.version_hardware[i];
  }

  for (size_t i = 0; i < 6; ++i) {
    state_info_msg.mac[i] = state_info.data.mac[i];
  }

  state_info_msg.cur_work_state = state_info.data.cur_work_state;
  state_info_msg.core_temp = state_info.data.core_temp;
  state_info_msg.powerup_cnt = state_info.data.powerup_cnt;
  state_info_msg.local_time_now = state_info.data.local_time_now;
  state_info_msg.last_sync_time = state_info.data.last_sync_time;
  state_info_msg.time_offset = state_info.data.time_offset;
  state_info_msg.time_sync_type = state_info.data.time_sync_type;

  // for (size_t i = 0; i < 32; ++i) {
  //   state_info_msg.status_code[i] = state_info.data.status_code[i];
  // }

  state_info_msg.lidar_diag_status = state_info.data.lidar_diag_status;
  // state_info_msg.lidar_flash_status = state_info.data.lidar_flash_status;
  state_info_msg.fw_type = state_info.data.fw_type;

  for (size_t i = 0; i < 8; ++i) {
    state_info_msg.hms_code[i] = state_info.data.hms_code[i];
  }

  // state_info_msg.ROI_Mode = state_info.data.ROI_Mode;
  PublisherPtr publisher_ptr = GetCurrentStateInfoPublisher(index);

  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(state_info_msg);
  }
}

PublisherPtr Lddc::GetCurrentPublisher(uint8_t index) {
  ros::Publisher **pub = nullptr;
  uint32_t queue_size = kMinEthPacketQueueSize;

  if (use_multi_topic_) {
    pub = &private_pub_[index];
    queue_size = queue_size / 8; // queue size is 4 for only one lidar
  } else {
    pub = &global_pub_;
    queue_size = queue_size * 8; // shared queue size is 256, for all lidars
  }

  if (*pub == nullptr) {
    char name_str[48];
    memset(name_str, 0, sizeof(name_str));
    if (use_multi_topic_) {
      std::string ip_string = IpNumToString(lds_->lidars_[index].handle);
      snprintf(name_str, sizeof(name_str), "livox/lidar_%s",
               ReplacePeriodByUnderline(ip_string).c_str());
      DRIVER_INFO(*cur_node_, "Support multi topics.");
    } else {
      DRIVER_INFO(*cur_node_, "Support only one topic.");
      snprintf(name_str, sizeof(name_str), "livox/lidar");
    }

    *pub = new ros::Publisher;
    if (kPointCloud2Msg == transfer_format_) {
      **pub =
          cur_node_->getMyPrivateNodeHandle().advertise<sensor_msgs::PointCloud2>(name_str, queue_size);
      DRIVER_INFO(*cur_node_,
          "%s publish use PointCloud2 format, set ROS publisher queue size %d",
          name_str, queue_size);
    } else if (kLivoxCustomMsg == transfer_format_) {
      **pub = cur_node_->getMyPrivateNodeHandle().advertise<livox_ros_driver2::CustomMsg>(name_str,
                                                                queue_size);
      DRIVER_INFO(*cur_node_,
          "%s publish use livox custom format, set ROS publisher queue size %d",
          name_str, queue_size);
    } else if (kPclPxyziMsg == transfer_format_) {
      **pub = cur_node_->getMyPrivateNodeHandle().advertise<PointCloud>(name_str, queue_size);
      DRIVER_INFO(*cur_node_,
          "%s publish use pcl PointXYZI format, set ROS publisher queue "
          "size %d",
          name_str, queue_size);
    }
  }

  return *pub;
}

PublisherPtr Lddc::GetCurrentImuPublisher(uint8_t handle) {
  ros::Publisher **pub = nullptr;
  uint32_t queue_size = kMinEthPacketQueueSize;

  if (use_multi_topic_) {
    pub = &private_imu_pub_[handle];
    queue_size = queue_size * 2; // queue size is 64 for only one lidar
  } else {
    pub = &global_imu_pub_;
    queue_size = queue_size * 8; // shared queue size is 256, for all lidars
  }

  if (*pub == nullptr) {
    char name_str[48];
    memset(name_str, 0, sizeof(name_str));
    if (use_multi_topic_) {
      DRIVER_INFO(*cur_node_, "Support multi topics.");
      std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
      snprintf(name_str, sizeof(name_str), "livox/imu_%s",
               ReplacePeriodByUnderline(ip_string).c_str());
    } else {
      DRIVER_INFO(*cur_node_, "Support only one topic.");
      snprintf(name_str, sizeof(name_str), "livox/imu");
    }

    *pub = new ros::Publisher;
    **pub = cur_node_->getMyPrivateNodeHandle().advertise<sensor_msgs::Imu>(name_str, queue_size);
    DRIVER_INFO(*cur_node_, "%s publish imu data, set ROS publisher queue size %d", name_str,
             queue_size);
  }

  return *pub;
}


PublisherPtr Lddc::GetCurrentStateInfoPublisher(uint8_t handle) {
  ros::Publisher **pub = nullptr;
  uint32_t queue_size = kMinEthPacketQueueSize;

  if (use_multi_topic_) {
    pub = &private_state_info_pub_[handle];
    queue_size = queue_size * 2; // queue size is 64 for only one lidar
  } else {
    pub = &global_state_info_pub_;
    queue_size = queue_size * 8; // shared queue size is 256, for all lidars
  }

  if (*pub == nullptr) {
    char name_str[48];
    memset(name_str, 0, sizeof(name_str));
    if (use_multi_topic_) {
      DRIVER_INFO(*cur_node_, "Support multi topics.");
      std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
      snprintf(name_str, sizeof(name_str), "livox/info_%s",
               ReplacePeriodByUnderline(ip_string).c_str());
    } else {
      DRIVER_INFO(*cur_node_, "Support only one topic.");
      snprintf(name_str, sizeof(name_str), "livox/info");
    }

    *pub = new ros::Publisher;
    **pub = cur_node_->getMyPrivateNodeHandle().advertise<livox_ros_driver2::StateInfoMsg>(name_str, queue_size);
    DRIVER_INFO(*cur_node_, "%s publish state info, set ROS publisher queue size %d", name_str,
             queue_size);
  }

  return *pub;
}

void Lddc::CreateBagFile(const std::string &file_name) {
  if (!bag_) {
    bag_ = new rosbag::Bag;
    bag_->open(file_name, rosbag::bagmode::Write);
    DRIVER_INFO(*cur_node_, "Create bag file :%s!", file_name.c_str());
  }
}

}  // namespace livox_ros
