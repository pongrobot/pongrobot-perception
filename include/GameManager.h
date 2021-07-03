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
#include <std_msgs/Empty.h>

enum GameState
{
    CALIBRATING,
    RESTARTING,
    IDLE,
    AIMING,
    SHOOTING
};

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
        ros::Subscriber shot_fired_sub_;
        ros::Subscriber calibration_request_sub_;
        ros::Subscriber calibration_complete_sub_;
        ros::Subscriber restart_request_sub_;
        ros::Publisher target_cup_pub_;
        ros::Publisher detector_restart_pub_;
        ros::Publisher detector_calibrate_pub_;
        
        // TF data
        std::string target_frame_id_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // Callbacks
        void cupArrayCallback( const geometry_msgs::PoseArray::ConstPtr& msg );
        void hasBallCallback( const std_msgs::Bool::ConstPtr& msg );
        void shotFiredCallback( const geometry_msgs::PoseStamped::ConstPtr& msg );
        void calibrationRequestCallback( const std_msgs::Empty::ConstPtr& msg );
        void calibrationCompleteCallback( const std_msgs::Empty::ConstPtr& msg );
        void restartRequestCallback( const std_msgs::Empty::ConstPtr& msg );

        // Msg data
        geometry_msgs::PoseArray detection_array_;
        geometry_msgs::PoseStamped target_cup_;
        int target_seq_id_;
        bool has_ball_;
        bool shot_success_;
        bool calibration_request_;
        bool calibration_complete_;
        bool restart_request_;

        // Game data
        GameState state_; 
        ros::Time command_sent_;
        ros::Time calibration_sent_;
        ros::Duration launcher_timeout_;
        ros::Duration calibration_timeout_;
        double cup_height_;
        
        // Utilities
        bool pickTarget();
};

#endif
