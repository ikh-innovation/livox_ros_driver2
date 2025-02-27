#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <pluginlib/class_list_macros.h>
#include "livox_ros_driver2/CustomMsg.h"
#include "livox_ros_driver2/CustomPoint.h"

namespace livox_ros
{
    class PCL2LivoxNodelet : public nodelet::Nodelet
    {
    public:
        virtual void onInit()
        {
            ros::NodeHandle &nh = getNodeHandle();
            ros::NodeHandle &private_nh = getPrivateNodeHandle();

            // Subscribe to input point cloud
            sub_ = nh.subscribe("input_point_cloud", 1, &PCL2LivoxNodelet::pointCloudCallback, this);

            // Advertise output point cloud
            pub_ = nh.advertise<livox_ros_driver2::CustomMsg>("output_point_cloud", 1);
        }

    private:
        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
        {
            livox_ros_driver2::CustomMsgPtr output{new livox_ros_driver2::CustomMsg()};

            output->header = cloud_msg->header;
            output->point_num = cloud_msg->width;
            output->timebase = cloud_msg->header.stamp.toNSec();
            output->points.resize(cloud_msg->width);

            const uint8_t* data_ptr = cloud_msg->data.data();
            const auto& fields = cloud_msg->fields;
            const size_t point_step = cloud_msg->point_step;

            for (size_t i = 0; i < cloud_msg->width; ++i)
            {
                const uint8_t* point_ptr = data_ptr + i * point_step;
                output->points[i].x = *reinterpret_cast<const float*>(point_ptr + fields[0].offset);
                output->points[i].y = *reinterpret_cast<const float*>(point_ptr + fields[1].offset);
                output->points[i].z = *reinterpret_cast<const float*>(point_ptr + fields[2].offset);
                output->points[i].reflectivity = static_cast<uint8_t>(*reinterpret_cast<const float*>(point_ptr + fields[3].offset));
                output->points[i].tag = *reinterpret_cast<const uint8_t*>(point_ptr + fields[4].offset);
                output->points[i].line = *reinterpret_cast<const uint8_t*>(point_ptr + fields[5].offset);
                output->points[i].offset_time = *reinterpret_cast<const uint32_t*>(point_ptr + fields[6].offset);
            }

            pub_.publish(output);
        }
      
        ros::Publisher pub_;
        ros::Subscriber sub_;
        sensor_msgs::PointCloud2Ptr points_msg_;              
    };
} // namespace livox_ros

PLUGINLIB_EXPORT_CLASS(livox_ros::PCL2LivoxNodelet, nodelet::Nodelet)