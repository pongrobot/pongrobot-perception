#include "ros/ros.h"
#include "CupDetector.h"


int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "cup_detection_node");
    ros::NodeHandle nh;

    // Create cup detector
    CupDetector cup_detector(nh);

    while (ros::ok())
    {
        cup_detector.run(); // Run the detector
        ros::spinOnce();
    }

    return 0;
}
