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

#include <pcl/common/time.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// #include <pcl/visualization/pcl_visualizer.h>

#include <sensor_msgs/PointCloud2.h>
// #include <nav_msgs/Odometry.h>

// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <dynamic_reconfigure/server.h>

namespace obstacle_detection
{

class Detector : public nodelet::Nodelet
{

public:
    virtual void onInit() override;
        
private:
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    pcl::PCLPointCloud2::Ptr accumulate_scans(const uint num_scans);
    
    ros::Subscriber sub_point_cloud_;

    std::mutex queue_mutex_;
    std::atomic<bool> need_pcl{false};
    std::condition_variable queue_cond_;
    std::queue<pcl::PCLPointCloud2::Ptr> scan_queue_;

};

} // namespace obstacle_detection
#endif // OBSTACLE_DETECTION