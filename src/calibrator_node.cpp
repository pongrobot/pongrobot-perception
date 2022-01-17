#include "ros/ros.h"
#include "Calibrator.h"


int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle nh;

    float loop_frq;
    if ( !nh.getParam("/rate/calibrator", loop_frq) )
    {
        ROS_ERROR("CalibrationNode cannot load param: /rate/calibrator");
    }
    ros::Rate loop_rate(loop_frq);

    // Create cup detector
    Calibrator calibrator(nh);

    while (ros::ok())
    {
        calibrator.run(); // Run the calibrator
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
