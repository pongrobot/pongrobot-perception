#include "ros/ros.h"
#include "SocketTelemetryNode.h"

int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "socket_telemetry_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    // Create the SocketTelemetryNode object
    SocketTelemetryNode socket_telemetry_node(nh);
    socket_telemetry_node.start();

    // Main loop
    while (ros::ok())
    {
        socket_telemetry_node.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    socket_telemetry_node.stop();

    return 0;
}
