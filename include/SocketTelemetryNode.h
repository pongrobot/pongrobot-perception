#ifndef SOCKET_TELEMETRY_NODE_H
#define SOCKET_TELEMETRY_NODE_H

// ROS headers
#include "ros/ros.h"
#include <tf2_ros/buffer_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

// WebSocket headers
#include "server_ws.hpp"
#include <future>
#include <memory>

using WsServer = SimpleWeb::SocketServer<SimpleWeb::WS>;

class SocketTelemetryNode
{
    public:
        SocketTelemetryNode( ros::NodeHandle nh );
        void start();
        void update();
        void stop();

    private:
        // Ros data
        ros::NodeHandle nh_;

        // WebSocket server
        WsServer server;
        std::unique_ptr<std::thread> serverThread;
        std::promise<unsigned short> server_port;
};

#endif
