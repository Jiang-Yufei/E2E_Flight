#ifndef _DATA_COLLECTION_MANAGER_H
#define _DATA_COLLECTION_MANAGER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>

namespace data_collection
{
class DataCollectionManager{

public:
    DataCollectionManager();
    ~DataCollectionManager();

    void init(ros::NodeHandle &nh);
    void CollectionTimerCallback(const ros::TimerEvent &e);
    void DepthImageCallback(const sensor_msgs::ImageConstPtr &msg);
    void DroneStateCallback(const nav_msgs::Odometry::ConstPtr &pose_msg);

    ros::Timer data_collection_timer_;
    ros::Subscriber depth_image_sub_, drone_state_sub;

    // Store the data and only need the latest one.
    sensor_msgs::Image depth_image_;
    geometry_msgs::PoseStamped drone_state_;
    // Store the twists
    geometry_msgs::TwistStamped drone_twist_;

    // Path of the data storage
    std::string depth_image_path_ = "/home/jyf/ROS_project/End_to_End_Flight/src/data_collection/data/image";
    std::string drone_state_path_ = "/home/jyf/ROS_project/End_to_End_Flight/src/data_collection/data/state";

};
}

#endif