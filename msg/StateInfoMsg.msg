uint32 pcl_data_type
uint32 pattern_mode
# uint32 dual_emit_en
# uint32 point_send_en
IpConfig lidar_ipcfg
HostIpConfig info_host_ipcfg
HostIpConfig pointcloud_host_ipcfg
HostIpConfig imu_host_ipcfg
# HostIpConfig ctl_host_ipcfg
# HostIpConfig log_host_ipcfg
# int32 vehicle_speed
# int32 environment_temp
InstallAttitude install_attitude
# uint32 blind_spot_set
# uint32 frame_rate
FovConfig fov_cfg0
FovConfig fov_cfg1
uint32 fov_cfg_en
uint32 detect_mode
uint32[4] func_io_cfg
uint32 work_tgt_mode
# uint32 glass_heat
uint32 imu_data_en
# uint32 fusa_en
string sn
string product_info
uint32[4] version_app
uint32[4] version_loader
uint32[4] version_hardware
uint32[6] mac
uint32 cur_work_state
int32 core_temp
uint32 powerup_cnt
uint64 local_time_now
uint64 last_sync_time
int64 time_offset
uint32 time_sync_type
# uint32[32] status_code
uint32 lidar_diag_status
# uint32 lidar_flash_status
uint32 fw_type
uint32[8] hms_code
# uint32 ROI_Mode