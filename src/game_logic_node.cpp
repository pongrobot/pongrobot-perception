#include "ros/ros.h"
#include "GameManager.h"

int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "game_logic_node");
    ros::NodeHandle nh;

    float loop_frq;
    nh.getParam("/rate/game", loop_frq);
    ros::Rate loop_rate(loop_frq);


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
