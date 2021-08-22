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
#include <std_msgs/Float32.h>

// WebSocket headers
#include "client_ws.hpp"
#include "server_ws.hpp"
#include <future>
#include <memory>

// MessagePack and JSON
#include "messagepack.h"
#include <nlohmann/json.hpp>

using WsServer = SimpleWeb::SocketServer<SimpleWeb::WS>;
using WsClient = SimpleWeb::SocketClient<SimpleWeb::WS>;

using json = nlohmann::json;

// Command types coming in from the UI
enum SocketCommandType {
    COMMAND,
    PARAMETER
};

enum SocketTelemetry {
    TELEMETRY,
    PARAMETERS,
    POINT_CLOUD
};

class SocketTelemetryNode
{
    public:
        SocketTelemetryNode( ros::NodeHandle nh );
        void start();
        void update();
        void stop();
        void handleCommand(json& jsonMsg);

    private:
        // Ros data
        ros::NodeHandle nh_;
        ros::Publisher trigger_pub_;
        ros::Publisher rpm_cmd_pub_;
        ros::Publisher zero_gimbal_pub_;

        // WebSocket server
        WsServer server;
        std::unique_ptr<std::thread> serverThread;
        std::promise<unsigned short> server_port;
};

#endif
