#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver2/CustomMsg.h"
#include <pluginlib/class_list_macros.h>

namespace livox_ros
{

class PclAnalyzerNodelet : public nodelet::Nodelet
{
public:
    virtual void onInit()
    {
        ros::NodeHandle& nh = getNodeHandle();
        ros::NodeHandle& nhp = getPrivateNodeHandle();

        // Get type of message to subscribe to
        int type = nhp.param("msg_type", 0); // 0: PointCloud2, 1: CustomMsg
        if (type != 0 && type != 1)
        {
            NODELET_ERROR("Invalid msg_type parameter. Use 0 for PointCloud2 or 1 for CustomMsg.");
            return;
        }

        // Get parameters
        pcl_timeout_ = nhp.param("pcl_topic_timeout", 10.0);
        minimum_points_ = nhp.param("user_few_points_number_threshold", 4000);
        minimum_points_time_ = nhp.param("user_few_points_time_threshold", 10.0);

        // Publishers
        pub1_ = nh.advertise<std_msgs::Bool>("enough", 10, true);
        pub2_ = nh.advertise<std_msgs::Bool>("active", 10, true);

        active_.data = true;
        enough_points_.data = true;

        // Initialize timers
        active_timer_ = nh.createTimer(ros::Duration(pcl_timeout_), &PclAnalyzerNodelet::activeCallback, this);
        enough_timer_ = nh.createTimer(ros::Duration(minimum_points_time_), &PclAnalyzerNodelet::enoughCallback, this);
        active_timer_.start();
        enough_timer_.start();       

        // Subscribe to the appropriate topic based on the message type
        if (type == 0)
        {
            sub_ = nh.subscribe("input_point_cloud", 10, &PclAnalyzerNodelet::pclCallback, this);
        }
        else
        {
            sub_ = nh.subscribe("input_point_cloud", 10, &PclAnalyzerNodelet::customCallback, this);
        }        
    }

private:
    void activeCallback(const ros::TimerEvent& event)
    {
        if (active_.data)
        {
            active_.data = false;
            pub2_.publish(active_);
        }
    }

    void enoughCallback(const ros::TimerEvent& event)
    {
        if (enough_points_.data)
        {
            enough_points_.data = false;
            pub1_.publish(enough_points_);
        }
    }

    void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        active_timer_.stop();
        if (!active_.data)
        {
            active_.data = true;
            pub2_.publish(active_);
        }

        if (msg->width * msg->height > minimum_points_)
        {
            enough_timer_.stop();
            if (!enough_points_.data)
            {
                enough_points_.data = true;
                pub1_.publish(enough_points_);
            }
        }
        else if (!enough_timer_.hasStarted())
        {
            enough_timer_.start();
        }
        
        active_timer_.start();
    }

    void customCallback(const livox_ros_driver2::CustomMsg::ConstPtr& msg)
    {
        active_timer_.stop();
        if (!active_.data)
        {
            active_.data = true;
            pub2_.publish(active_);
        }

        if (msg->point_num > minimum_points_)
        {
            enough_timer_.stop();
            if (!enough_points_.data)
            {
                enough_points_.data = true;
                pub1_.publish(enough_points_);
            }
        }
        else if (!enough_timer_.hasStarted())
        {
            enough_timer_.start();
        }
        
        active_timer_.start();
    }

    ros::Publisher pub1_;
    ros::Publisher pub2_;
    ros::Subscriber sub_;

    ros::Timer active_timer_;
    ros::Timer enough_timer_;

    std_msgs::Bool active_;
    std_msgs::Bool enough_points_;

    int minimum_points_;
    double pcl_timeout_;
    double minimum_points_time_;
};

} // namespace livox_ros

PLUGINLIB_EXPORT_CLASS(livox_ros::PclAnalyzerNodelet, nodelet::Nodelet)
