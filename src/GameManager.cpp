#include "GameManager.h"

GameManager::
GameManager( ros::NodeHandle nh ):
    tf_listener_(tf_buffer_),
    target_seq_id(0),
    has_ball_(false)
{
    nh_ = nh;

    // Pull in config data
    nh.param<std::string>("launcher_frame_id", target_frame_id_, "launcher");
    nh.param<double>("cup_height", cup_height_, 0.01);

    cup_array_sub_ = nh_.subscribe<geometry_msgs::PoseArray>("/detector/cup_array", 1, &GameManager::cupArrayCallback, this);
    has_ball_sub_ = nh_.subscribe<std_msgs::Bool>("/launcher/has_ball", 1, &GameManager::hasBallCallback, this);
    target_cup_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/launcher/target_pose", 100);  

    target_cup_.reset( new geometry_msgs::PoseStamped() );
}

void
GameManager::
cupArrayCallback( const geometry_msgs::PoseArray::ConstPtr& msg )
{
    detection_array_ = *msg;
}

void
GameManager::
hasBallCallback( const std_msgs::Bool::ConstPtr& msg )
{
    has_ball_ = msg->data;
}

bool
GameManager::
pickTarget( )
{
    bool target_found = false;

    if ( !detection_array_.poses.empty() )
    {
        // Set up target header
        target_cup_->header.seq = target_seq_id;
        target_cup_->header.stamp = ros::Time::now();
        target_cup_->header.frame_id = detection_array_.header.frame_id;
        target_seq_id++;

        // TODO: implement a real algorithm to pick the best target cup
        target_cup_->pose = detection_array_.poses[0];

        // Transform the the command to launcher frame
        try
        {
            // wait for the correct transform for up to 3 sec
            geometry_msgs::PoseStamped launcher_cmd = tf_buffer_.transform(*target_cup_, target_frame_id_, ros::Duration(3.0) ); 
            target_cup_->pose.position.z += cup_height_/2.f; // assuming detection is the centroid of the cup, aim for the top
            target_cup_pub_.publish(launcher_cmd);
            target_found = true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
        } 
    }

    return target_found;
}

void
GameManager::
run()
{
    // Check for a ball, then calculate command and send to launcher
    if ( has_ball_ )
    {
        if ( pickTarget() )
        {
            // target has been found, send launcher command      
            has_ball_ = false;
        }
    }
}
