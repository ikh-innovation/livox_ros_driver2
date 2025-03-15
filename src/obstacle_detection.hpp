#ifndef OBSTACLE_DETECTION 
#define OBSTACLE_DETECTION

#include <mutex>
#include <queue>
#include <atomic>
#include <thread>
#include <condition_variable>
// #include <cassert>
// #include <cstddef>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>

#include <pcl_ros/point_cloud.h>

// #include <pcl/visualization/pcl_visualizer.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver2/DetectorConfig.h"

#include <rviz_visual_tools/rviz_visual_tools.h>


// #include <nav_msgs/Odometry.h>


// #include <tf2_ros/transform_broadcaster.h>
// #include <dynamic_reconfigure/server.h>

namespace obstacle_detection
{

class Detector : public nodelet::Nodelet
{

public:
    ~Detector();
    virtual void onInit() override;

private:
    void process();
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulate_scans(const uint num_scans, const double max_range);
    void configCallback(livox_ros_driver2::DetectorConfig &config, uint32_t level);

    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    rviz_visual_tools::RvizVisualToolsPtr visual_tools2_;

    
    ros::Publisher test_pub_;
    ros::Publisher test_pub2_;
    ros::Publisher test_pub3_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_point_cloud_;

    Eigen::Vector3f perpendicular_to_ground_{0.0, 0.0, 1.0};
    Eigen::Isometry3f base_to_lidar_{Eigen::Isometry3f::Identity()};
    Eigen::Isometry3f pose1_;
    Eigen::Isometry3f pose2_;
    
    uint min_pts_voxel_;
    uint num_scans_;
    uint max_iterations_;
    uint min_plane_points_;
    double distance_threshold_;
    double eps_angle_;
    double max_height_;
    double model_variance_threshold_;
    double max_range_;
    double max_obstacle_height_;

    bool enable_;
    bool use_imu_{false};
    double voxel_size_;
    std::mutex imu_mutex_;
    std::mutex conf_mutex_;
    std::mutex queue_mutex_;
    std::thread process_thread_;
    std::atomic<bool> need_pcl_{false};
    std::atomic<bool> stop_thread_{false};
    std::condition_variable conf_cond_;
    std::condition_variable queue_cond_;
    std::queue<pcl::PointCloud<pcl::PointXYZI>::Ptr> scan_queue_;    
    std::shared_ptr<dynamic_reconfigure::Server<livox_ros_driver2::DetectorConfig>> server_;
};

} // namespace obstacle_detection
#endif // OBSTACLE_DETECTION