#ifndef TF_BROADCAST_NODE_H
#define TF_BROADCAST_NODE_H

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/PoseStamped.h"

class TFBroadcastNode
{
    public:
        struct FrameIDs
        {
            std::string camera_frame_id;
            std::string world_frame_id;
            std::string robot_base_frame_id;
            std::string robot_center_frame_id;
            std::string launcher_frame_id;
        };

        struct GeometryOffsets
        {
            double cam_x_offset;
            double cam_z_offset;
            double launcher_z_offset;
        };

    public:
        TFBroadcastNode( ros::NodeHandle nh );
        void loadParams();

    private:
        // ROS Data
        ros::NodeHandle nh_;
        ros::Subscriber imu_sub_;

        // Params
        FrameIDs        frame_ids_;
        GeometryOffsets geometry_offsets_;

        geometry_msgs::Quaternion identity_;

        // Transforms
        geometry_msgs::TransformStamped world_robot_base_tf_;
        geometry_msgs::TransformStamped robot_base_center_tf_;
        geometry_msgs::TransformStamped center_camera_tf_;
        geometry_msgs::TransformStamped center_launcher_tf_;

    private:
        void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);

};

#endif