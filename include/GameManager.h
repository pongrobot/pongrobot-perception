#ifndef GAME_MANAGER_H
#define GAME_MANAGER_H

// ROS headers
#include "ros/ros.h"
#include <tf2_ros/buffer_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

class GameManager
{
    public:
        GameManager( ros::NodeHandle nh );
        void run();

    private:
        // Ros data
        ros::NodeHandle nh_;
        ros::Subscriber has_ball_sub_;
        ros::Subscriber cup_array_sub_;
        ros::Publisher target_cup_pub_;
        
        // TF data
        std::string target_frame_id_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // Callbacks
        void cupArrayCallback( const geometry_msgs::PoseArray::ConstPtr& msg );
        void hasBallCallback( const std_msgs::Bool::ConstPtr& msg );

        // Msg data
        geometry_msgs::PoseArray detection_array_;
        geometry_msgs::PoseStamped::Ptr target_cup_;
        int target_seq_id;
        bool has_ball_;
        
        // Utilities
        bool pickTarget();
};

#endif
