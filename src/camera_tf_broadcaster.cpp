#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"

ros::Publisher stamped_pub;

double cam_x = 0;
double cam_y = 0;
double cam_z = 0;
std::string camera_frame_id = "camera_link"; 
std::string robot_frame_id = "brobot"; 

void orientationCallback(const geometry_msgs::Quaternion& msg)
{
    // broadcast camera transform 
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = robot_frame_id;
    transformStamped.child_frame_id = camera_frame_id;
    transformStamped.transform.translation.x = cam_x;
    transformStamped.transform.translation.y = cam_y;
    transformStamped.transform.translation.z = cam_z;
    transformStamped.transform.rotation = msg;
    br.sendTransform(transformStamped);

    // Populate the PoseStamped with the orientation from the IMU and the position from the config
    geometry_msgs::PoseStamped camera_pose;
    camera_pose.header.stamp = ros::Time::now();
    camera_pose.header.frame_id = robot_frame_id;
    camera_pose.pose.orientation = msg;

    stamped_pub.publish(camera_pose);
    ROS_INFO("Published Stamped Pose");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_tf2_broadcaster");
    ros::NodeHandle n;

    // Read cam position from the config
    n.getParam("cam_x", cam_x);
    n.getParam("cam_y", cam_y);
    n.getParam("cam_z", cam_z);
    n.getParam("camera_frame_id", camera_frame_id);
    n.getParam("robot_frame_id", robot_frame_id);
    
    ros::Subscriber sub = n.subscribe("imu_orientation", 1000, orientationCallback);
    stamped_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);
    ros::Rate loop_rate(5);

    // Main Loop
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
};
