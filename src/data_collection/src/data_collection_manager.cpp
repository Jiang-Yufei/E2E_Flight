#include <ros/ros.h>
#include "data_collection/data_collection_manager.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>

namespace data_collection
{
    DataCollectionManager::DataCollectionManager(){};
    DataCollectionManager::~DataCollectionManager(){};
    
    void DataCollectionManager::init(ros::NodeHandle &nh){
        data_collection_timer_ = nh.createTimer(ros::Duration(0.1), &DataCollectionManager::CollectionTimerCallback, this);
        depth_image_sub_ = nh.subscribe("/camera/depth/image_raw", 1, &DataCollectionManager::DepthImageCallback, this);
        drone_state_sub = nh.subscribe("/vins_estimator/odometry", 1, &DataCollectionManager::DroneStateCallback, this);
    }

    void DataCollectionManager::CollectionTimerCallback(const ros::TimerEvent& e){
        // Store the latest depth image to .png file, the name of the file is the time stamp
        if (depth_image_.data.size() > 0){
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(depth_image_, sensor_msgs::image_encodings::TYPE_16UC1);
            }
            catch (cv_bridge::Exception &e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            std::string file_name = depth_image_path_ + "/" + std::to_string(depth_image_.header.stamp.toSec()) + ".png";
            cv::imwrite(file_name, cv_ptr->image);
            //ROS_INFO("Save depth image to %s", file_name.c_str());
        }
        // Store the latest drone state into .txt file, each frame of data is in a line
        if (drone_state_.pose.position.x != 0){
            std::string file_name = drone_state_path_ + "/" + "state.txt";
            std::ofstream file(file_name, std::ofstream::app);
            file << drone_state_.pose.position.x << " " << drone_state_.pose.position.y << " " << drone_state_.pose.position.z << " "\
            << drone_state_.pose.orientation.x << " " << drone_state_.pose.orientation.y << " " << drone_state_.pose.orientation.z << " "\
            << drone_twist_.twist.linear.x << " " << drone_twist_.twist.linear.y << " " << drone_twist_.twist.linear.z << " "\
            << drone_twist_.twist.angular.x << " " << drone_twist_.twist.angular.y << " " << drone_twist_.twist.angular.z \
            << std::endl;
            file.close();
            ROS_INFO("Save drone state to %s", file_name.c_str());
        }


    }

    void DataCollectionManager::DepthImageCallback(const sensor_msgs::ImageConstPtr &msg){
        depth_image_ = *msg;
    }

    void DataCollectionManager::DroneStateCallback(const nav_msgs::Odometry::ConstPtr &pose_msg){
        drone_state_.pose = pose_msg->pose.pose;
        drone_twist_.twist = pose_msg->twist.twist;
    }

}