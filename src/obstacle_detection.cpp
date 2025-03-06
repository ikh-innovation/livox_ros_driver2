#include <obstacle_detection.hpp>
#include <pluginlib/class_list_macros.h>

// #include <std_msgs/Float64.h>
// #include <pcl/common/time.h>
// #include <pcl_ros/transforms.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/conditional_removal.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/segmentation/sac_segmentation.h>

namespace obstacle_detection
{

void Detector::onInit()
{
    ROS_INFO("Detector constructor");

    ros::NodeHandle &nh_ = getMTNodeHandle();
    ros::NodeHandle &private_nh_ = getMTPrivateNodeHandle();

    // Subscribers
    sub_point_cloud_ = nh_.subscribe("input_pointcloud", 1, &Detector::pointCloudCallback, this);
};

void Detector::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    ROS_INFO("pointCloudCallback");

    if (need_pcl.load())
    {
        ROS_INFO("Getting point cloud");
        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(*msg, *cloud);

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            scan_queue_.push(cloud);
        }
        queue_cond_.notify_one();
    }
    else
    {
        ROS_INFO("Ignoring point cloud");
    }
}

pcl::PCLPointCloud2::Ptr Detector::accumulate_scans(const uint num_scans)
{
    ROS_INFO("accumulate_scans");

    pcl::StopWatch watch;
    watch.reset();

    // Pointer for final point cloud
    pcl::PCLPointCloud2::Ptr final_cloud{new pcl::PCLPointCloud2};

    // Empty queue
    std::queue<pcl::PCLPointCloud2::Ptr> empty;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        std::swap(scan_queue_, empty);
    }

    need_pcl.store(true);
    
    // Append point clouds as they come in
    uint i = 0;
    while (i < num_scans && ros::ok())
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cond_.wait(lock, [this] { return !scan_queue_.empty() || !ros::ok(); });

        if (!ros::ok())
        {
            return final_cloud;
        }

        // *final_cloud += *scan_queue_.front();
        // scan_queue_.pop();
        i++;
    }

    need_pcl.store(false);

    return final_cloud;
}

} // namespace obstacle_detection

PLUGINLIB_EXPORT_CLASS(obstacle_detection::Detector, nodelet::Nodelet)