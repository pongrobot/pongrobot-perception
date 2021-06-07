#include "ros/ros.h"
#include "GameManager.h"

int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "game_logic_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10); // run loop at 10Hz

    // Create the GameManager object
    GameManager game_manager(nh);

    // Main loop
    while (ros::ok())
    {
        game_manager.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
