#include "TFBroadcastNode.h"

TFBroadcastNode::
TFBroadcastNode
(ros::NodeHandle nh)
{
    nh_ = nh;
    loadParams();

    // Init subscriber
    imu_sub_ = nh_.subscribe<geometry_msgs::Pose> ("imu_pose", 1, &TFBroadcastNode::poseCallback, this);

    // Initialize identity quaternion
    identity_.w = 1.f;
    identity_.x = 0.f;
    identity_.y = 0.f;
    identity_.z = 0.f;

    // Initialize world -> robot base transform 
    world_robot_base_tf_.header.frame_id = frame_ids_.world_frame_id;
    world_robot_base_tf_.child_frame_id = frame_ids_.robot_base_frame_id;
    world_robot_base_tf_.transform.translation.x = 0.f;
    world_robot_base_tf_.transform.translation.y = 0.f;
    world_robot_base_tf_.transform.translation.z = 0.f;

    // Initialize robot base -> robot center transform
    robot_base_center_tf_.header.frame_id = frame_ids_.robot_base_frame_id;
    robot_base_center_tf_.child_frame_id = frame_ids_.robot_center_frame_id;
    robot_base_center_tf_.transform.translation.x = 0.f;
    robot_base_center_tf_.transform.translation.y = 0.f;
    robot_base_center_tf_.transform.rotation = identity_;

    // Initialize robot center -> camera transform
    center_camera_tf_.header.frame_id = frame_ids_.robot_center_frame_id;
    center_camera_tf_.child_frame_id = frame_ids_.camera_frame_id;
    center_camera_tf_.transform.translation.y = 0.f;
    center_camera_tf_.transform.rotation = identity_;

    // Initialize robot center -> launcher transform
    center_launcher_tf_.header.frame_id = frame_ids_.robot_center_frame_id;
    center_launcher_tf_.child_frame_id = frame_ids_.launcher_frame_id;
    center_launcher_tf_.transform.translation.x = 0.f;
    center_launcher_tf_.transform.translation.y = 0.f;
    center_launcher_tf_.transform.rotation = identity_;

}

void
TFBroadcastNode::
loadParams()
{
    // Load geometry params
    if ( !nh_.getParam("geometry/camera_x_offset", geometry_offsets_.cam_x_offset) )
    {
        ROS_ERROR("Brobot TF Node cannot load param: %s/geometry/camera_x_offset", nh_.getNamespace().c_str());
    }
    if ( !nh_.getParam("geometry/camera_z_offset", geometry_offsets_.cam_z_offset) )
    {
        ROS_ERROR("Brobot TF Node cannot load param: %s/geometry/camera_z_offset", nh_.getNamespace().c_str());
    }
    if ( !nh_.getParam("geometry/launcher_z_offset", geometry_offsets_.launcher_z_offset) )
    {
        ROS_ERROR("Brobot TF Node cannot load param: %s/geometry/launcher_z_offset", nh_.getNamespace().c_str());
    }

    // Load frame ID params
    if ( !nh_.getParam("frame/camera_frame_id", frame_ids_.camera_frame_id) )
    {
        ROS_ERROR("Brobot TF Node cannot load param: %s/frame/camera_frame_id", nh_.getNamespace().c_str());
    }
    if ( !nh_.getParam("frame/world_frame_id", frame_ids_.world_frame_id) )
    {
        ROS_ERROR("Brobot TF Node cannot load param: %s/frame/world_frame_id", nh_.getNamespace().c_str());
    }
    if ( !nh_.getParam("frame/robot_center_frame_id", frame_ids_.robot_center_frame_id) )
    {
        ROS_ERROR("Brobot TF Node cannot load param: %s/frame/robot_center_frame_id", nh_.getNamespace().c_str());
    }
    if ( !nh_.getParam("frame/robot_base_frame_id", frame_ids_.robot_base_frame_id) )
    {
        ROS_ERROR("Brobot TF Node cannot load param: %s/frame/robot_base_frame_id", nh_.getNamespace().c_str());
    }
    if ( !nh_.getParam("frame/launcher_frame_id", frame_ids_.launcher_frame_id) )
    {
        ROS_ERROR("Brobot TF Node cannot load param: %s/frame/launcher_frame_id", nh_.getNamespace().c_str());
    } 
}

/////////////////////////////////////////////////////////
// Callback
/////////////////////////////////////////////////////////
void
TFBroadcastNode::
poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    static tf2_ros::StaticTransformBroadcaster br;

    // Extract Yaw
    tf2::Quaternion sensor_orientation;
    tf2::convert(msg->orientation, sensor_orientation);
    sensor_orientation.normalize();
    
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(sensor_orientation);
    m.getRPY(roll, pitch, yaw);

    tf2::Quaternion yaw_only;
    yaw_only.setRPY( 0, 0 , yaw );

    tf2::Quaternion tf_quaternion;
    tf_quaternion = sensor_orientation * yaw_only.inverse();
    tf_quaternion.normalize();
    geometry_msgs::Quaternion rp_orientation = tf2::toMsg(tf_quaternion);

    // Update world -> robot base transform
    world_robot_base_tf_.header.stamp = ros::Time::now();
    world_robot_base_tf_.transform.rotation = rp_orientation;
    br.sendTransform(world_robot_base_tf_);

    // Update robot base -> robot center transform
    robot_base_center_tf_.header.stamp = ros::Time::now();
    robot_base_center_tf_.transform.translation.z = msg->position.z;
    br.sendTransform(robot_base_center_tf_);

    // Update robot center -> camera transform 
    center_camera_tf_.header.stamp = ros::Time::now();
    center_camera_tf_.transform.translation.x = geometry_offsets_.cam_x_offset;
    center_camera_tf_.transform.translation.z = geometry_offsets_.cam_z_offset;
    br.sendTransform(center_camera_tf_);

    // Update robot center -> launcher transform
    center_launcher_tf_.header.stamp = ros::Time::now();
    center_launcher_tf_.transform.translation.z = geometry_offsets_.launcher_z_offset;
    br.sendTransform(center_launcher_tf_);
}

/////////////////////////////////////////////////////////
// Main
/////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "tf_broadcast_node");
    ros::NodeHandle nh;

     // Load loop rate
    float loop_frq;
    if ( !nh.getParam("rate", loop_frq) )
    {
        ROS_ERROR("TF Broadcaster Node cannot load param: %s/rate", nh.getNamespace().c_str());
    }

    ros::Rate loop_rate(loop_frq);
    TFBroadcastNode tf_broadcast_node(nh);

    while (ros::ok())
    {
        ros::spin();
        //loop_rate.sleep();
    }
}