#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher stamped_pub;

double cam_x = 0;
double cam_z = 0;
double launcher_z = 0;

std::string camera_frame_id = "camera_link"; 
std::string world_frame_id = "world"; 
std::string robot_base_frame_id = "robot_base"; 
std::string robot_center_frame_id = "robot_center"; 
std::string launcher_frame_id = "launcher"; 

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    static tf2_ros::StaticTransformBroadcaster br;

    // Extract Yaw
    double roll, pitch, yaw;
    tf2::Quaternion sensor_orientation;
    tf2::convert(msg->orientation, sensor_orientation);
    tf2::Matrix3x3 m(sensor_orientation);
    m.getRPY(roll, pitch, yaw);
    tf2::Quaternion yaw_only;
    yaw_only.setRPY( 0, 0 , yaw );

    // Remove yaw component from tf
    tf2::Quaternion tf_quaternion;
    tf_quaternion = sensor_orientation * yaw_only.inverse();
    geometry_msgs::Quaternion rp_orientation = tf2::toMsg(tf_quaternion);
    
    // Identity Quaternion
    geometry_msgs::Quaternion identity;
    identity.w = 1.f;

    // broadcast world -> robot base transform 
    geometry_msgs::TransformStamped world_robot_base_tf;
    world_robot_base_tf.header.stamp = ros::Time::now();
    world_robot_base_tf.header.frame_id = world_frame_id;
    world_robot_base_tf.child_frame_id = robot_base_frame_id;
    world_robot_base_tf.transform.translation.x = 0.f;
    world_robot_base_tf.transform.translation.y = 0.f;
    world_robot_base_tf.transform.translation.z = 0.f;
    world_robot_base_tf.transform.rotation = rp_orientation;
    br.sendTransform(world_robot_base_tf);

    // broadcast robot base -> robot center transform 
    geometry_msgs::TransformStamped robot_base_center_tf;
    robot_base_center_tf.header.stamp = ros::Time::now();
    robot_base_center_tf.header.frame_id = robot_base_frame_id;
    robot_base_center_tf.child_frame_id = robot_center_frame_id;
    robot_base_center_tf.transform.translation.x = 0.f;
    robot_base_center_tf.transform.translation.y = 0.f;
    robot_base_center_tf.transform.translation.z = msg->position.z;
    robot_base_center_tf.transform.rotation = identity;
    br.sendTransform(robot_base_center_tf);
    
    // broadcast robot center -> camera transform 
    geometry_msgs::TransformStamped center_camera_tf;
    center_camera_tf.header.stamp = ros::Time::now();
    center_camera_tf.header.frame_id = robot_center_frame_id;
    center_camera_tf.child_frame_id = camera_frame_id;
    center_camera_tf.transform.translation.x = cam_x;
    center_camera_tf.transform.translation.y = 0.f;
    center_camera_tf.transform.translation.z = cam_z;
    center_camera_tf.transform.rotation = identity;
    br.sendTransform(center_camera_tf);

    // broadcast robot center -> launcher transform 
    geometry_msgs::TransformStamped center_launcher_tf;
    center_launcher_tf.header.stamp = ros::Time::now();
    center_launcher_tf.header.frame_id = robot_center_frame_id;
    center_launcher_tf.child_frame_id = launcher_frame_id;
    center_launcher_tf.transform.translation.z = launcher_z;
    center_launcher_tf.transform.rotation = identity;
    br.sendTransform(center_launcher_tf);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pongrobot_tf2_broadcaster");
    ros::NodeHandle n;
    float loop_frq;

    // Read cam position from the config
    if ( !n.getParam("/rate/tf_broadcast", loop_frq) )
    {
        ROS_ERROR("PongrobotTFBroadcaster cannot load param: /rate/tf_broadcast");
    }

    if ( !n.getParam("/frame/geometry/camera_x_offset", cam_x) )
    {
        ROS_ERROR("PongrobotTFBroadcaster cannot load param: /frame/geometry/camera_x_offset");
    }

    if ( !n.getParam("/frame/geometry/camera_z_offset", cam_z) )
    {
        ROS_ERROR("PongrobotTFBroadcaster cannot load param: /frame/geometry/camera_z_offset");
    }
    
    if ( !n.getParam("/frame/geometry/launcher_z_offset", launcher_z) )
    {
        ROS_ERROR("PongrobotTFBroadcaster cannot load param: /frame/geometry/launcher_z_offset");
    }

    if ( !n.getParam("/frame/camera_frame_id", camera_frame_id) )
    {
        ROS_ERROR("PongrobotTFBroadcaster cannot load param: /frame/camera_frame_id");
    }

    if ( !n.getParam("/frame/world_frame_id", world_frame_id) )
    {
        ROS_ERROR("PongrobotTFBroadcaster cannot load param: /frame/world_frame_id");
    }

    if ( !n.getParam("/frame/robot_center_frame_id", robot_center_frame_id) )
    {
        ROS_ERROR("PongrobotTFBroadcaster cannot load param: /frame/robot_center_frame_id");
    }

    if ( !n.getParam("/frame/robot_base_frame_id", robot_base_frame_id) )
    {
        ROS_ERROR("PongrobotTFBroadcaster cannot load param: /frame/robot_base_frame_id");
    }

    if ( !n.getParam("/frame/launcher_frame_id", launcher_frame_id) )
    {
        ROS_ERROR("PongrobotTFBroadcaster cannot load param: /frame/launcher_frame_id");
    } 

    ros::Subscriber sub = n.subscribe("imu_pose", 1000, poseCallback);
    ros::Rate loop_rate(loop_frq);

    // Main Loop
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
};
