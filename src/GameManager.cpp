#include "GameManager.h"

GameManager::
GameManager( ros::NodeHandle nh ):
    tf_listener_(tf_buffer_),
    target_seq_id_(0),
    has_ball_(false),
    shot_success_(false),
    calibration_request_(false),
    restart_request_(false),
    state_(GameState::IDLE) //TODO: Initialize to calibrating when setup
{
    nh_ = nh;
    
    // Pull in config data
    double launcher_timeout_val;
    double calibration_timeout_val;

    nh.param<std::string>("/frame/launcher_frame_id", target_frame_id_, "launcher");
    nh.param<double>("cup_height", cup_height_, 0.1);
    nh.param<double>("launcher_timeout", launcher_timeout_val, 10.0);
    nh.param<double>("calibration_timeout", calibration_timeout_val, 30.0);

    launcher_timeout_ = ros::Duration(launcher_timeout_val);    
    calibration_timeout_ = ros::Duration(calibration_timeout_val);    

    // Setup Publishers and Subscribers
    cup_array_sub_ = nh_.subscribe<geometry_msgs::PoseArray>("/detector/cup_array", 1, &GameManager::cupArrayCallback, this);
    has_ball_sub_ = nh_.subscribe<std_msgs::Bool>("/launcher/has_ball", 1, &GameManager::hasBallCallback, this);
    shot_fired_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/launcher/shot", 1, &GameManager::shotFiredCallback, this);
    calibration_request_sub_ = nh.subscribe<std_msgs::Empty>("/calibration/request", 1, &GameManager::calibrationRequestCallback, this);
    calibration_complete_sub_ = nh.subscribe<std_msgs::Empty>("/calibration/complete", 1, &GameManager::calibrationCompleteCallback, this);
    restart_request_sub_ = nh.subscribe<std_msgs::Empty>("/restart", 1, &GameManager::restartRequestCallback, this);
    target_cup_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/launcher/target_pose", 100);  
    detector_restart_pub_ = nh_.advertise<std_msgs::Empty>("/detector/restart", 100);  
    detector_calibrate_pub_ = nh_.advertise<std_msgs::Empty>("/detector/calibrate", 100);  
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

void
GameManager::
shotFiredCallback( const geometry_msgs::PoseStamped::ConstPtr& msg )
{
    // TODO: Verify that confirmation is from the correct shot
    shot_success_ = true;
}

void
GameManager::
calibrationRequestCallback( const std_msgs::Empty::ConstPtr& msg )
{
    calibration_request_ = true;
}

void
GameManager::
calibrationCompleteCallback( const std_msgs::Empty::ConstPtr& msg )
{
    calibration_complete_ = true;
}

void
GameManager::
restartRequestCallback( const std_msgs::Empty::ConstPtr& msg )
{
    restart_request_ = true;
}

bool
GameManager::
pickTarget( )
{
    bool target_found = false;

    if ( !detection_array_.poses.empty() )
    {
        // Set up target header
        target_cup_.header.seq = target_seq_id_;
        target_cup_.header.stamp = ros::Time::now();
        target_cup_.header.frame_id = detection_array_.header.frame_id;
        target_seq_id_++;

        // TODO: implement a real algorithm to pick the best target cup
        target_cup_.pose = detection_array_.poses[0];

        // Transform the the command to launcher frame
        try
        {
            // wait for the correct transform for up to 3 sec
            target_cup_ = tf_buffer_.transform(target_cup_, target_frame_id_, ros::Duration(3.0) ); 
            target_cup_.pose.position.z += cup_height_/2.f; // assuming detection is the centroid of the cup, aim for the top
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
    switch(state_)
    {
        case GameState::CALIBRATING:
        { 
            if (calibration_complete_)
            {
                state_ = GameState::RESTARTING;
                ROS_INFO("[GameManager] CALIBRATING->RESTARTING: Calibration completed sucessfully");
            }
            else if (ros::Time::now() - calibration_sent_ > calibration_timeout_)
            {
                // TODO: reload default params if possible
                state_ = GameState::RESTARTING;
                ROS_WARN("[GameManager] CALIBRATING->RESTARTING: Calibration timed out, using default values");
            }

            break;
        }
        case GameState::RESTARTING:
        {
            // Send reboot request to the CupDetector node
            std_msgs::Empty restart;
            detector_restart_pub_.publish(restart);

            state_ = GameState::IDLE;
            ROS_INFO("[GameManager] RESTARTING->IDLE: Restart complete");
            break;
        }
        case GameState::IDLE:
        {
            // Process calibration request
            if (calibration_request_)
            {
                // Request calibration
                std_msgs::Empty request;
                detector_calibrate_pub_.publish(request);             
                calibration_sent_ = ros::Time::now();

                // Reset processed flags
                calibration_request_ = false;
                calibration_complete_ = false;
                restart_request_ = false; // Process restart command concurrently
                state_ = GameState::CALIBRATING;
                ROS_INFO("[GameManager] IDLE->CALIBRATING: Received calibration request");
            }
            
            // Process restart request
            if (restart_request_)
            {
                restart_request_ = false;
                state_ = GameState::RESTARTING;
                ROS_INFO("[GameManager] IDLE->RESTARTING: Received restart request");
            }

            // Check for ball
            if ( has_ball_ )
            {
                state_ = GameState::AIMING;
                ROS_INFO("[GameManager] IDLE->AIMING: Ball detected");
            }

            break;
        }
        case GameState::AIMING:
        {
            bool target_found = pickTarget();
        
            if ( target_found )
            {
                target_cup_pub_.publish(target_cup_);
                state_ = GameState::SHOOTING;
                command_sent_ = ros::Time::now();
                shot_success_ = false;
                has_ball_ = false;
                ROS_INFO("[GameManager] Sending Command: %.4f, %.4f, %.4f [%s]", 
                                                    target_cup_.pose.position.x, 
                                                    target_cup_.pose.position.y, 
                                                    target_cup_.pose.position.z,
                                                    target_cup_.header.frame_id.c_str());

                ROS_INFO("[GameManager] AIMING->SHOOTING: Target found");
            }
            else
            {
                ROS_WARN("[GameManager] AIMING->IDLE: Target NOT found");
                state_ = GameState::IDLE;
            }

            break;
        }
        case GameState::SHOOTING:
        {
            if (shot_success_)
            {
                ROS_INFO("[GameManager] SHOOTING->IDLE: Received shot confirmation");
                state_ = GameState::IDLE;  
            }
            else if  ( ros::Time::now() - command_sent_ > launcher_timeout_ )
            {
                ROS_WARN("[GameManager] SHOOTING->IDLE: Timed out waiting for launcher confirmation");
                state_ = GameState::IDLE;  
            }
            break;
        } 
    }
}
