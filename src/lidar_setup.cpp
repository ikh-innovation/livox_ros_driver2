#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <memory>
#include <atomic>

// Shared variable to track success
std::shared_ptr<std::atomic<bool>> setup_success = std::make_shared<std::atomic<bool>>(false);

void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data) 
{
    if (response == nullptr) {
      return;
    }
    printf("RebootCallback, status:%u, handle:%u, ret_code:%u",
        status, handle, response->ret_code);
    
    // Check if the call failed
    if (response->ret_code != 0) {
        auto success_flag = static_cast<std::atomic<bool>*>(client_data);
        *success_flag = false;  // Mark setup as failed
    }
}

void AsyncControlCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) 
{
    if (response == nullptr) {
      return;
    }
    printf("AsyncCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
        status, handle, response->ret_code, response->error_key);

    // Check if the call failed
    if (response->ret_code != 0) {
        auto success_flag = static_cast<std::atomic<bool>*>(client_data);
        *success_flag = false;  // Mark setup as failed
    }
  
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) 
{
    if (info == nullptr) {
      printf("lidar info change callback failed, the info is nullptr.\n");
      return;
    }
    *setup_success = true;  // Mark setup as successful 
    printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);
    
    printf("Setting work mode");
    SetLivoxLidarWorkMode(handle, kLivoxLidarWakeUp, AsyncControlCallback, setup_success.get());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    printf("Setting work mode after boot");
    SetLivoxLidarWorkModeAfterBoot(handle, kLivoxLidarWorkModeAfterBootWakeUp, AsyncControlCallback, setup_success.get());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    printf("Setting point cloud data type");
    SetLivoxLidarPclDataType(handle, kLivoxLidarCartesianCoordinateHighData, AsyncControlCallback, setup_success.get());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    printf("Setting scan pattern");
    SetLivoxLidarScanPattern(handle, kLivoxLidarScanPatternNoneRepetive, AsyncControlCallback, setup_success.get());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    printf("Setting dual emit");
    SetLivoxLidarDualEmit(handle, false, AsyncControlCallback, setup_success.get());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    printf("Setting detect mode");
    SetLivoxLidarDetectMode(handle, kLivoxLidarDetectNormal, AsyncControlCallback, setup_success.get());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    printf("Setting install attitude");
    LivoxLidarInstallAttitude install_attitude;
    install_attitude.roll_deg = 0.0;
    install_attitude.pitch_deg = 0.0;
    install_attitude.yaw_deg = 0.0;
    install_attitude.x = 0;
    install_attitude.y = 0;
    install_attitude.z = 0;
    SetLivoxLidarInstallAttitude(handle, &install_attitude, AsyncControlCallback, setup_success.get());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    printf("Setting FOV cfg0");
    FovCfg fov_cfg0;
    fov_cfg0.yaw_start = 0.0;
    fov_cfg0.yaw_stop = 0.0;
    fov_cfg0.pitch_start = -7.0;
    fov_cfg0.pitch_stop = 52.0;
    SetLivoxLidarFovCfg0(handle, &fov_cfg0, AsyncControlCallback, setup_success.get());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    printf("Setting FOV cfg1");
    FovCfg fov_cfg1;
    fov_cfg1.yaw_start = 0.0;
    fov_cfg1.yaw_stop = 0.0;
    fov_cfg1.pitch_start = -7.0;
    fov_cfg1.pitch_stop = 52.0;
    SetLivoxLidarFovCfg1(handle, &fov_cfg1, AsyncControlCallback, setup_success.get());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    printf("Disabling FOV");
    DisableLivoxLidarFov(handle, AsyncControlCallback, setup_success.get());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    printf("Enabling IMU data");
    EnableLivoxLidarImuData(handle, AsyncControlCallback, setup_success.get());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    printf("Setting Lidar IP");
    LivoxLidarIpInfo lidar_ip_info;
    strcpy(lidar_ip_info.ip_addr, "192.168.2.151");
    strcpy(lidar_ip_info.net_mask, "255.255.255.0");
    strcpy(lidar_ip_info.gw_addr, "192.168.2.1");
    SetLivoxLidarIp(handle, &lidar_ip_info, AsyncControlCallback, setup_success.get());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    printf("Setting state info host IP");
    HostStateInfoIpInfo host_state_info;
    strcpy(host_state_info.host_ip_addr, "192.168.2.50");
    host_state_info.host_state_info_port = 56201;
    host_state_info.lidar_state_info_port = 56200;
    SetLivoxLidarStateInfoHostIPCfg(handle, &host_state_info, AsyncControlCallback, setup_success.get());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    printf("Setting point cloud data host IP");
    HostPointIPInfo host_point_ip_info;
    strcpy(host_point_ip_info.host_ip_addr, "192.168.2.50");
    host_point_ip_info.host_point_data_port = 56301;
    host_point_ip_info.lidar_point_data_port = 56300;
    SetLivoxLidarPointDataHostIPCfg(handle, &host_point_ip_info, AsyncControlCallback, setup_success.get());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    printf("Setting IMU data host IP");
    HostImuDataIPInfo host_imu_ip_info;
    strcpy(host_imu_ip_info.host_ip_addr, "192.168.2.50");
    host_imu_ip_info.host_imu_data_port = 56401;
    host_imu_ip_info.lidar_imu_data_port = 56400;
    SetLivoxLidarImuDataHostIPCfg(handle, &host_imu_ip_info, AsyncControlCallback, setup_success.get());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    printf("Setting function IO config");
    FuncIOCfg func_io_cfg;
    func_io_cfg.in0 = 0;
    func_io_cfg.int1 = 0;
    func_io_cfg.out0 = 0;
    func_io_cfg.out1 = 0;
    SetLivoxLidarFuncIOCfg(handle, &func_io_cfg, AsyncControlCallback, setup_success.get());   
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // printf("Setting blind spot");
    // SetLivoxLidarBlindSpot
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    printf("Rebooting LiDAR");
    LivoxLidarRequestReboot(handle, RebootCallback, setup_success.get());
}

int main(int argc, const char *argv[]) 
{
    const std::string path = argv[1];

    // REQUIRED, to init Livox SDK2
    if (!LivoxLidarSdkInit(path.c_str())) {
        printf("Livox Init Failed\n");
        LivoxLidarSdkUninit();
        return -1;
    }

    // REQUIRED, to get a handle to targeted lidar and set its work mode to NORMAL
    SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

    // sleep
    std::this_thread::sleep_for(std::chrono::seconds(10));

    LivoxLidarSdkUninit();

    // Check if setup was successful
    if (!*setup_success) {
        printf("Lidar setup failed!\n");
        return -1;
    }

    printf("Lidar setup succeeded!\n");
    return 0;
}