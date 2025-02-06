#include <mutex>
#include <ros/ros.h>
#include <Eigen/Geometry> 
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/transform_listener.h>
#include <pluginlib/class_list_macros.h>
#include "livox_ros_driver2/CustomMsg.h"
#include "livox_ros_driver2/CustomPoint.h"
#include <geometry_msgs/TransformStamped.h>
#include "livox_ros_driver2/CustomFilterConfig.h"

namespace livox_ros
{
    class CustomFilterNodelet : public nodelet::Nodelet
    {
    public:
        virtual void onInit()
        {
            ros::NodeHandle &nh = getNodeHandle();
            ros::NodeHandle &private_nh = getPrivateNodeHandle();

            // Initialize TF listener
            tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

            // Set up dynamic reconfigure
            server_ = std::make_shared<dynamic_reconfigure::Server<livox_ros_driver2::CustomFilterConfig>>(mutex_, private_nh);
            dynamic_reconfigure::Server<livox_ros_driver2::CustomFilterConfig>::CallbackType f;
            f = boost::bind(&CustomFilterNodelet::configCallback, this, _1, _2);
            server_->setCallback(f);

            // Subscribe to input point cloud
            sub_ = nh.subscribe("input_point_cloud", 1, &CustomFilterNodelet::pointCloudCallback, this);

            // Advertise output point cloud
            pub_ = nh.advertise<livox_ros_driver2::CustomMsg>("filtered_point_cloud", 1);
        }

    private:
        void configCallback(livox_ros_driver2::CustomFilterConfig &config, uint32_t level)
        {
            boost::recursive_mutex::scoped_lock lock(mutex_);
            enable_ = config.enable;
            box_min_x_ = config.box_min_x;
            box_min_y_ = config.box_min_y;
            box_min_z_ = config.box_min_z;
            box_max_x_ = config.box_max_x;
            box_max_y_ = config.box_max_y;
            box_max_z_ = config.box_max_z;
            if (config.box_frame != box_frame_)
                tf_ready_ = false;
            box_frame_ = config.box_frame;
        }

        void pointCloudCallback(const livox_ros_driver2::CustomMsgConstPtr &cloud_msg)
        {
            livox_ros_driver2::CustomMsgPtr output{new livox_ros_driver2::CustomMsg(*cloud_msg)};
            boost::recursive_mutex::scoped_lock lock(mutex_);
            if (enable_)
            {
                if (!tf_ready_)
                {
                    try
                    {
                        if (cloud_msg->header.frame_id != box_frame_)
                        {
                            geometry_msgs::TransformStamped t_in = tf_buffer_.lookupTransform(box_frame_, cloud_msg->header.frame_id, ros::Time(0));
                            box_to_lidar_tf_ = Eigen::Affine3f(Eigen::Translation3f(t_in.transform.translation.x, t_in.transform.translation.y,
                                                                                    t_in.transform.translation.z) * Eigen::Quaternionf(
                                                                                        t_in.transform.rotation.w, t_in.transform.rotation.x,
                                                                                        t_in.transform.rotation.y, t_in.transform.rotation.z));
                        }
                        else
                        {
                            box_to_lidar_tf_ = Eigen::Affine3f::Identity();
                        }
                        tf_ready_ = true;
                    }
                    catch (tf2::TransformException &ex)
                    {
                        ROS_WARN_THROTTLE(5, "Transform failed: %s", ex.what());
                        return;
                    }
                }            
                
                // Transform and filter points
                auto it = std::remove_if(output->points.begin(), output->points.end(), [&](livox_ros_driver2::CustomPoint &p) {
                    Eigen::Vector3f point = box_to_lidar_tf_ * Eigen::Vector3f(p.x, p.y, p.z);
                    if (point.x() < box_min_x_ || point.y() < box_min_y_ || point.z() < box_min_z_ || point.x() > box_max_x_ || point.y() > box_max_y_ || point.z() > box_max_z_)
                    {
                        // If outside the cropbox, update the point
                        p.x = point.x();
                        p.y = point.y();
                        p.z = point.z();
                        return false;
                    }
                    else
                    {
                        // If inside the cropbox, remove the point
                        return true;
                    }
                });
                output->points.erase(it, output->points.end());
                output->header.frame_id = box_frame_;
                output->point_num = output->points.size();
            }
            pub_.publish(output);
        }

        boost::recursive_mutex mutex_;
        bool enable_{true};
        bool tf_ready_{false};
        ros::Publisher pub_;
        ros::Subscriber sub_;      
        std::string box_frame_;
        tf2_ros::Buffer tf_buffer_;
        Eigen::Affine3f box_to_lidar_tf_;      
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        double box_min_x_, box_min_y_, box_min_z_, box_max_x_, box_max_y_, box_max_z_;
        std::shared_ptr<dynamic_reconfigure::Server<livox_ros_driver2::CustomFilterConfig>> server_;        
    };
} // namespace livox_ros

PLUGINLIB_EXPORT_CLASS(livox_ros::CustomFilterNodelet, nodelet::Nodelet)