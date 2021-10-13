#include "ros/ros.h"
#include "CupDetector.h"


int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "cup_detection_node");
    ros::NodeHandle nh;

    float loop_frq;
    if ( !nh.getParam("/rate/detector", loop_frq) )
    {
        ROS_ERROR("CupDetector cannot load param: /rate/detector");
    }
    ros::Rate loop_rate(loop_frq);


    // Create cup detector
    CupDetector cup_detector(nh);

    while (ros::ok())
    {
        cup_detector.run(); // Run the detector
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
