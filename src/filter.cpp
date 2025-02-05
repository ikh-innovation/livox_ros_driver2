#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace livox_ros
{
    class FilterNodelet : public nodelet::Nodelet
    {
    public:
        virtual void onInit()
        {
            ros::NodeHandle &nh = getNodeHandle();
            ros::NodeHandle &private_nh = getPrivateNodeHandle();

            // Get parameters
            private_nh.param("box_min_x", box_min_x_, -1.0);
            private_nh.param("box_min_y", box_min_y_, -1.0);
            private_nh.param("box_min_z", box_min_z_, -1.0);
            private_nh.param("box_max_x", box_max_x_, 1.0);
            private_nh.param("box_max_y", box_max_y_, 1.0);
            private_nh.param("box_max_z", box_max_z_, 1.0);
            private_nh.param("box_frame", box_frame_, std::string("box_frame"));

            // Initialize TF listener
            tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

            // Subscribe to input point cloud
            sub_ = nh.subscribe("input_point_cloud", 1, &FilterNodelet::pointCloudCallback, this);

            // Advertise output point cloud
            pub_ = nh.advertise<sensor_msgs::PointCloud2>("filtered_point_cloud", 1);
        }

    private:
        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
        {
            // Transform point cloud to box frame
            sensor_msgs::PointCloud2 output{*cloud_msg};
            if (cloud_msg->header.frame_id != box_frame_)
            {
                while (!tf_ready_)
                {
                    try
                    {
                        box_to_lidar_tf_ = tf_buffer_.lookupTransform(box_frame_, cloud_msg->header.frame_id, ros::Time(0));
                        tf_ready_ = true;
                    }
                    catch (tf2::TransformException &ex)
                    {
                        ROS_WARN_THROTTLE(5, "Transform failed: %s", ex.what());
                    }
                }
                tf2::doTransform(output, output, box_to_lidar_tf_);
            }


            pcl::PCLPointCloud2::Ptr output_cloud(new pcl::PCLPointCloud2);
            pcl_conversions::toPCL(output, *output_cloud);            

            pcl::CropBox<pcl::PCLPointCloud2> box_filter;
            box_filter.setMin(Eigen::Vector4f(box_min_x_, box_min_y_, box_min_z_, 1.0));
            box_filter.setMax(Eigen::Vector4f(box_max_x_, box_max_y_, box_max_z_, 1.0));
            box_filter.setInputCloud(output_cloud);
            box_filter.setNegative(true);
            box_filter.filter(*output_cloud);

            pcl_conversions::moveFromPCL(*output_cloud, output);
            // output.header = cloud_msg->header;
            // output.header.frame_id = box_frame_;
            pub_.publish(output);
        }
        
        bool tf_ready_;
        ros::Publisher pub_;
        ros::Subscriber sub_;      
        std::string box_frame_;
        tf2_ros::Buffer tf_buffer_;        
        geometry_msgs::TransformStamped box_to_lidar_tf_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        double box_min_x_, box_min_y_, box_min_z_, box_max_x_, box_max_y_, box_max_z_;
    };
} // namespace livox_ros

PLUGINLIB_EXPORT_CLASS(livox_ros::FilterNodelet, nodelet::Nodelet)