#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <pluginlib/class_list_macros.h>
#include "livox_ros_driver2/CustomMsg.h"

namespace livox_ros
{
    class Livox2PCL2Nodelet : public nodelet::Nodelet
    {
    public:
        virtual void onInit()
        {
            ros::NodeHandle &nh = getNodeHandle();
            ros::NodeHandle &private_nh = getPrivateNodeHandle();

            points_msg_.reset(new sensor_msgs::PointCloud2);

            const auto add_field = [this](const std::string& name, const int offset, const uint8_t datatype) {
                sensor_msgs::PointField field;
                field.name = name;
                field.offset = offset;
                field.datatype = datatype;
                field.count = 1;
                points_msg_->fields.push_back(field);
            };

            add_field("x", 0, sensor_msgs::PointField::FLOAT32);
            add_field("y", points_msg_->fields.back().offset + sizeof(float), sensor_msgs::PointField::FLOAT32);
            add_field("z", points_msg_->fields.back().offset + sizeof(float), sensor_msgs::PointField::FLOAT32);
            add_field("intensity", points_msg_->fields.back().offset + sizeof(float), sensor_msgs::PointField::FLOAT32);            
            add_field("tag", points_msg_->fields.back().offset + sizeof(float), sensor_msgs::PointField::UINT8);
            add_field("line", points_msg_->fields.back().offset + sizeof(std::uint8_t), sensor_msgs::PointField::UINT8);
            add_field("timestamp", points_msg_->fields.back().offset + sizeof(std::uint8_t), sensor_msgs::PointField::FLOAT64);
            points_msg_->is_bigendian = false;
            points_msg_->point_step = sizeof(float) * 4 + sizeof(uint8_t) * 2 + sizeof(double);
            points_msg_->is_dense = true;

            // Subscribe to input point cloud
            sub_ = nh.subscribe("input_point_cloud", 1, &Livox2PCL2Nodelet::pointCloudCallback, this);

            // Advertise output point cloud
            pub_ = nh.advertise<sensor_msgs::PointCloud2>("output_point_cloud", 1);
        }

    private:
        void pointCloudCallback(const livox_ros_driver2::CustomMsgConstPtr &livox_msg)
        {
            points_msg_->header = livox_msg->header;
            points_msg_->width = livox_msg->point_num;
            points_msg_->height = 1;

            points_msg_->row_step = livox_msg->point_num * points_msg_->point_step;
            points_msg_->data.resize(points_msg_->row_step);

            unsigned char* ptr = points_msg_->data.data();
            for (int i = 0; i < livox_msg->point_num; i++) {
                *reinterpret_cast<float*>(ptr + points_msg_->fields[0].offset) = livox_msg->points[i].x;
                *reinterpret_cast<float*>(ptr + points_msg_->fields[1].offset) = livox_msg->points[i].y;
                *reinterpret_cast<float*>(ptr + points_msg_->fields[2].offset) = livox_msg->points[i].z;
                *reinterpret_cast<float*>(ptr + points_msg_->fields[3].offset) = livox_msg->points[i].reflectivity;
                *reinterpret_cast<std::uint8_t*>(ptr + points_msg_->fields[4].offset) = livox_msg->points[i].tag;
                *reinterpret_cast<std::uint8_t*>(ptr + points_msg_->fields[5].offset) = livox_msg->points[i].line;
                *reinterpret_cast<double*>(ptr + points_msg_->fields[6].offset) = livox_msg->timebase + livox_msg->points[i].offset_time;                

                ptr += points_msg_->point_step;
            }

            sensor_msgs::PointCloud2Ptr output{new sensor_msgs::PointCloud2(*points_msg_)};
           
            pub_.publish(output);
        }
      
        ros::Publisher pub_;
        ros::Subscriber sub_;
        sensor_msgs::PointCloud2Ptr points_msg_;              
    };
} // namespace livox_ros

PLUGINLIB_EXPORT_CLASS(livox_ros::Livox2PCL2Nodelet, nodelet::Nodelet)