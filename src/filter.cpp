#include <mutex>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <nodelet/nodelet.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/transform_listener.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include "livox_ros_driver2/FilterConfig.h"

namespace livox_ros
{
    class FilterNodelet : public nodelet::Nodelet
    {
    public:
        virtual void onInit()
        {
            ros::NodeHandle &nh = getNodeHandle();
            ros::NodeHandle &private_nh = getPrivateNodeHandle();

            // Initialize indexes                  
            const std::string resolved_topic{nh.resolveName("input_point_cloud", true)};
            sensor_msgs::PointCloud2ConstPtr init_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(resolved_topic);
            if (init_msg)
            {
                for (std::size_t d = 0; d < init_msg->fields.size (); ++d)
                {
                    if (init_msg->fields[d].name == "x")
                    {
                        x_idx_ = d;
                    }
                    else if (init_msg->fields[d].name == "y")
                    {
                        y_idx_ = d;
                    }
                    else if (init_msg->fields[d].name == "z")
                    {
                        z_idx_ = d;
                    }
                    else if (init_msg->fields[d].name == "tag")
                    {
                        tag_idx_ = d;
                    }
                }

                if (x_idx_ == -1 || y_idx_ == -1 || z_idx_ == -1 || tag_idx_ == -1)
                {
                    ROS_ERROR("Could not find required fields (x,y,z,tag) in point cloud message");
                    exit(1);
                }
                
                // Obtain the size of datatype
                const auto sizeofDatatype = [](const auto& datatype) -> int
                {
                    const auto size = pcl::getFieldSize(datatype);
                    if (size == 0) {
                        ROS_ERROR("Invalid field type (%d)!\n", datatype);
                    }
                    return size;
                };

                // Restrict size of a field to be at-max sizeof(FLOAT64) now to support {U}INT64
                field_sizes_.resize(init_msg->fields.size());
                std::transform(init_msg->fields.begin(), init_msg->fields.end(), field_sizes_.begin(),
                                [&sizeofDatatype](const auto& field)
                                {
                                    return std::min(sizeofDatatype(field.datatype), static_cast<int>(sizeof(double)));
                                });
            }
            else
            {
                ROS_ERROR("Could not get initial point cloud message");
                exit(1);
            }             

            // Initialize TF listener
            tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

            // Set up dynamic reconfigure
            server_ = std::make_shared<dynamic_reconfigure::Server<livox_ros_driver2::FilterConfig>>(mutex_, private_nh);
            dynamic_reconfigure::Server<livox_ros_driver2::FilterConfig>::CallbackType f;
            f = boost::bind(&FilterNodelet::configCallback, this, _1, _2);
            server_->setCallback(f);

            // Subscribe to input point cloud
            sub_ = nh.subscribe("input_point_cloud", 1, &FilterNodelet::pointCloudCallback, this);

            // Advertise output point cloud
            pub_ = nh.advertise<sensor_msgs::PointCloud2>("filtered_point_cloud", 1);
        }

    private:
        void configCallback(livox_ros_driver2::FilterConfig &config, uint32_t level)
        {
            boost::recursive_mutex::scoped_lock lock(mutex_);
            if (config.box_min_x > config.box_max_x || config.box_min_y > config.box_max_y || config.box_min_z > config.box_max_z ||
                config.outer_box_min_x > config.outer_box_max_x || config.outer_box_min_y > config.outer_box_max_y || config.outer_box_min_z > config.outer_box_max_z ||
                config.box_min_x < config.outer_box_min_x || config.box_min_y < config.outer_box_min_y || config.box_min_z < config.outer_box_min_z ||
                config.box_max_x > config.outer_box_max_x || config.box_max_y > config.outer_box_max_y || config.box_max_z > config.outer_box_max_z)
            {
                config.box_min_x = box_min_x_;
                config.box_min_y = box_min_y_;
                config.box_min_z = box_min_z_;
                config.box_max_x = box_max_x_;
                config.box_max_y = box_max_y_;
                config.box_max_z = box_max_z_;
                config.outer_box_min_x = outer_box_min_x_;
                config.outer_box_min_y = outer_box_min_y_;
                config.outer_box_min_z = outer_box_min_z_;
                config.outer_box_max_x = outer_box_max_x_;
                config.outer_box_max_y = outer_box_max_y_;
                config.outer_box_max_z = outer_box_max_z_;
            }
            enable_box_ = config.enable_box_filtering;
            enable_tag_ = config.enable_tag_filtering;
            box_min_x_ = config.box_min_x;
            box_min_y_ = config.box_min_y;
            box_min_z_ = config.box_min_z;
            box_max_x_ = config.box_max_x;
            box_max_y_ = config.box_max_y;
            box_max_z_ = config.box_max_z;
            outer_box_min_x_ = config.outer_box_min_x;
            outer_box_min_y_ = config.outer_box_min_y;
            outer_box_min_z_ = config.outer_box_min_z;
            outer_box_max_x_ = config.outer_box_max_x;
            outer_box_max_y_ = config.outer_box_max_y;
            outer_box_max_z_ = config.outer_box_max_z;
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

        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
        {
            sensor_msgs::PointCloud2Ptr output{new sensor_msgs::PointCloud2(*cloud_msg)};
            boost::recursive_mutex::scoped_lock lock(mutex_);
            if (enable_box_ || enable_tag_)
            {
                pcl::PCLPointCloud2::Ptr output_cloud(new pcl::PCLPointCloud2);
                pcl_conversions::toPCL(*output, *output_cloud);
                
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
                if (original_indices_.size () != (static_cast<std::size_t>(output_cloud->width) * static_cast<std::size_t>(output_cloud->height)))
                {
                    const auto indices_size = original_indices_.size ();
                    try
                    {
                        original_indices_.resize (static_cast<std::size_t>(output_cloud->width) * static_cast<std::size_t>(output_cloud->height));
                    }
                    catch (const std::bad_alloc&)
                    {
                        ROS_ERROR ("Failed to allocate %u indices.\n", (output_cloud->width * output_cloud->height));
                        return;
                    }
                    if (indices_size < original_indices_.size())
                        std::iota(original_indices_.begin() + indices_size, original_indices_.end(), indices_size);
                }
                
                std::vector<int> indices(output_cloud->width * output_cloud->height);
                int indices_count = 0;

                Eigen::Vector3f local_pt(Eigen::Vector3f::Zero ());

                for (const auto index : original_indices_)
                {
                    std::size_t point_offset = static_cast<std::size_t>(index) * output_cloud->point_step;
                    std::size_t offset;
                    if (enable_box_)
                    {
                        // Get local point                        
                        offset = point_offset + output_cloud->fields[x_idx_].offset;
                        memcpy(local_pt.data(), &output_cloud->data[offset], sizeof(float)*3);

                        local_pt = box_to_lidar_tf_ * local_pt;

                        // Keep if between the box and the outer box, otherwise continue
                        if ((local_pt.x() < outer_box_min_x_ || local_pt.y() < outer_box_min_y_ || local_pt.z() < outer_box_min_z_) ||
                            (local_pt.x() > outer_box_max_x_ || local_pt.y() > outer_box_max_y_ || local_pt.z() > outer_box_max_z_) ||
                            (local_pt.x() >= box_min_x_ && local_pt.y() >= box_min_y_ && local_pt.z() >= box_min_z_ && 
                            local_pt.x() <= box_max_x_ && local_pt.y() <= box_max_y_ && local_pt.z() <= box_max_z_))
                        {
                            continue;                 
                        }
                    }

                    if (enable_tag_)
                    {
                        offset = point_offset + output_cloud->fields[tag_idx_].offset;
                        uint8_t tag{output_cloud->data[offset]};
                        uint8_t drag_tag = tag & 0x03;
                        uint8_t atm_tag = (tag >> 2) & 0x03;
                        uint8_t other_tag = (tag >> 4) & 0x03;
                        uint8_t reserved_tag = (tag >> 6) & 0x03;
                        if (((other_tag == 0 && !other_high_) || (other_tag == 1 && !other_moderate_) || (other_tag == 2 && !other_low_) || (other_tag == 3 && !other_rsrv_)) ||
                            ((reserved_tag == 0 && !rsrv_zero_) || (reserved_tag == 1 && !rsrv_one_) || (reserved_tag == 2 && !rsrv_two_) || (reserved_tag == 3 && !rsrv_three_)) ||
                            ((atm_tag == 0 && !atm_high_) || (atm_tag == 1 && !atm_moderate_) || (atm_tag == 2 && !atm_low_) || (atm_tag == 3 && !atm_rsrv_)) ||
                            ((drag_tag == 0 && !drag_high_) || (drag_tag == 1 && !drag_moderate_) || (drag_tag == 2 && !drag_low_) || (drag_tag == 3 && !drag_rsrv_)))
                        {
                            continue;
                        }
                    }

                    indices[indices_count++] = index;
                }

                indices.resize (indices_count);                
                pcl::copyPointCloud(*output_cloud, indices, *output_cloud);
                pcl_conversions::moveFromPCL(*output_cloud, *output);
            }
            
            pub_.publish(output);
        }

        ros::Publisher pub_;
        ros::Subscriber sub_;

        tf2_ros::Buffer tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::shared_ptr<dynamic_reconfigure::Server<livox_ros_driver2::FilterConfig>> server_;

        Eigen::Affine3f box_to_lidar_tf_;

        std::string box_frame_;

        boost::recursive_mutex mutex_;
        std::vector<uint> field_sizes_;        
        std::vector<int> original_indices_; 

        int x_idx_{-1}, y_idx_{-1}, z_idx_{-1}, tag_idx_{-1};

        bool tf_ready_{false};
        bool enable_box_{true}, enable_tag_{true};
        bool rsrv_zero_{true}, rsrv_one_{true}, rsrv_two_{true}, rsrv_three_{true};        
        bool atm_high_{true}, atm_moderate_{true}, atm_low_{true}, atm_rsrv_{true};
        bool drag_high_{true}, drag_moderate_{true}, drag_low_{true}, drag_rsrv_{true};
        bool other_high_{true}, other_moderate_{true}, other_low_{true}, other_rsrv_{true};

        double box_min_x_, box_min_y_, box_min_z_, box_max_x_, box_max_y_, box_max_z_;
        double outer_box_min_x_, outer_box_min_y_, outer_box_min_z_, outer_box_max_x_, outer_box_max_y_, outer_box_max_z_;
    };
} // namespace livox_ros

PLUGINLIB_EXPORT_CLASS(livox_ros::FilterNodelet, nodelet::Nodelet)