#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

// Global to store the most recent cup array
geometry_msgs::PoseArray cup_array;
geometry_msgs::PoseStamped target_cup;
bool has_ball;

bool pickTarget()
{
    bool found_target = false;

    // Make sure you have at least one cup
    if ( !cup_array.poses.empty() )
    {
        // TODO: Ben, add logic to select the correct target cup
        target_cup.header = cup_array.header;
        target_cup.header.stamp = ros::Time();
        target_cup.pose = cup_array.poses[0];

        found_target = true;
    }

    return found_target;
}

void hasBallCallback( const std_msgs::Bool::ConstPtr& msg )
{
    has_ball = msg->data;
}

void cupArrayCallback( const geometry_msgs::PoseArray::ConstPtr& msg )
{
    // Store the array of cups (might want to keep the n most recent arrays)
    cup_array = *msg;
}

int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "game_logic_node");
    ros::NodeHandle nh;
    ros::Subscriber cup_array_sub = nh.subscribe<geometry_msgs::PoseArray>("/detector/cup_array", 1, cupArrayCallback);
    ros::Subscriber has_ball_sub = nh.subscribe<std_msgs::Bool>("/has_ball", 1, hasBallCallback);
    ros::Publisher target_cup_pub = nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 100);

    // Setup loop rate
    ros::Rate loop_rate(10); // run loop at 10Hz

    // Initialize state to no ball
    has_ball = false;

    while (ros::ok())
    {
        ros::spinOnce();
        bool target_found = pickTarget();

        if ( target_found && has_ball )
        {
            // if we have a target and a ball, request a shot
            target_cup_pub.publish(target_cup);
        }

        loop_rate.sleep();
    }

    return 0;
}
