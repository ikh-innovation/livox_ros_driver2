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
            enable_box_ = config.enable_box_filtering;
            enable_tag_ = config.enable_tag_filtering;
            box_min_x_ = config.box_min_x;
            box_min_y_ = config.box_min_y;
            box_min_z_ = config.box_min_z;
            box_max_x_ = config.box_max_x;
            box_max_y_ = config.box_max_y;
            box_max_z_ = config.box_max_z;
            if (config.box_frame != box_frame_)
                tf_ready_ = false;
            box_frame_ = config.box_frame;
            other_high_ = config.other_high;
            other_moderate_ = config.other_moderate;
            other_low_ = config.other_low;
            other_rsrv_ = config.other_rsrv;
            atm_high_ = config.atm_high;
            atm_moderate_ = config.atm_moderate;
            atm_low_ = config.atm_low;
            atm_rsrv_ = config.atm_rsrv;
            drag_high_ = config.drag_high;
            drag_moderate_ = config.drag_moderate;
            drag_low_ = config.drag_low;
            drag_rsrv_ = config.drag_rsrv;
            rsrv_zero_ = config.rsrv_zero;
            rsrv_one_ = config.rsrv_one;
            rsrv_two_ = config.rsrv_two;
            rsrv_three_ = config.rsrv_three;
        }

        void pointCloudCallback(const livox_ros_driver2::CustomMsgConstPtr &cloud_msg)
        {
            livox_ros_driver2::CustomMsgPtr output{new livox_ros_driver2::CustomMsg(*cloud_msg)};
            boost::recursive_mutex::scoped_lock lock(mutex_);
            if (enable_box_ || enable_tag_)
            {
                if (enable_box_ && !tf_ready_)
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
                    if (enable_box_)
                    {
                        Eigen::Vector3f point = box_to_lidar_tf_ * Eigen::Vector3f(p.x, p.y, p.z);
                        // If inside the cropbox, remove the point
                        if (point.x() >= box_min_x_ && point.y() >= box_min_y_ && point.z() >= box_min_z_ && 
                            point.x() <= box_max_x_ && point.y() <= box_max_y_ && point.z() <= box_max_z_)
                        {
                            return true;
                        }
                    }

                    if (enable_tag_)
                    {
                        uint8_t drag_tag = p.tag & 0x03;
                        uint8_t atm_tag = (p.tag >> 2) & 0x03;
                        uint8_t other_tag = (p.tag >> 4) & 0x03;
                        uint8_t reserved_tag = (p.tag >> 6) & 0x03;
                        if (((other_tag == 0 && !other_high_) || (other_tag == 1 && !other_moderate_) || (other_tag == 2 && !other_low_) || (other_tag == 3 && !other_rsrv_)) ||
                            ((reserved_tag == 0 && !rsrv_zero_) || (reserved_tag == 1 && !rsrv_one_) || (reserved_tag == 2 && !rsrv_two_) || (reserved_tag == 3 && !rsrv_three_)) ||
                            ((atm_tag == 0 && !atm_high_) || (atm_tag == 1 && !atm_moderate_) || (atm_tag == 2 && !atm_low_) || (atm_tag == 3 && !atm_rsrv_)) ||
                            ((drag_tag == 0 && !drag_high_) || (drag_tag == 1 && !drag_moderate_) || (drag_tag == 2 && !drag_low_) || (drag_tag == 3 && !drag_rsrv_)))
                        {
                            return true;
                        }
                    }

                    return false;
                    
                });
                output->points.erase(it, output->points.end());
                output->point_num = output->points.size();
            }
            pub_.publish(output);
        }

        boost::recursive_mutex mutex_;
        bool tf_ready_{false};
        bool enable_box_{true}, enable_tag_{true};
        bool rsrv_zero_{true}, rsrv_one_{true}, rsrv_two_{true}, rsrv_three_{true};        
        bool atm_high_{true}, atm_moderate_{true}, atm_low_{true}, atm_rsrv_{true};
        bool drag_high_{true}, drag_moderate_{true}, drag_low_{true}, drag_rsrv_{true};
        bool other_high_{true}, other_moderate_{true}, other_low_{true}, other_rsrv_{true};        
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