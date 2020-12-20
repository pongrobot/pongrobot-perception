#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"

ros::Publisher stamped_pub;

double cam_x = 0;
double cam_y = 0;
double cam_z = 0;

void orientationCallback(const geometry_msgs::Quaternion& msg)
{
    // Populate the PoseStamped with the orientation from the IMU and the position from the config
    geometry_msgs::PoseStamped imu_pose;
    imu_pose.header.stamp = ros::Time::now();
    imu_pose.header.frame_id = "map";
    imu_pose.pose.position.x = cam_x;
    imu_pose.pose.position.y = cam_y;
    imu_pose.pose.position.z = cam_z;
    imu_pose.pose.orientation = msg;

    stamped_pub.publish(imu_pose);
    ROS_INFO("Published Stamped Pose");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "orientation_to_pose");
  ros::NodeHandle n;

  // Read cam position from the config
  n.getParam("/cam_x", cam_x);
  n.getParam("/cam_y", cam_y);
  n.getParam("/cam_z", cam_z);

  ros::Subscriber sub = n.subscribe("imu_orientation", 1000, orientationCallback);
  stamped_pub = n.advertise<geometry_msgs::PoseStamped>("imu_pose", 1000);
  ros::Rate loop_rate(10);

  // Main Loop
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
