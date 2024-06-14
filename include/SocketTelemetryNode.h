#ifndef SOCKET_TELEMETRY_NODE_H
#define SOCKET_TELEMETRY_NODE_H

// ROS headers
#include "ros/ros.h"
#include <tf2_ros/buffer_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>

// PCL headers
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include <pcl/point_cloud.h>

// WebSocket headers
#include "client_ws.hpp"
#include "server_ws.hpp"
#include <future>
#include <memory>

// MessagePack and JSON
#include "messagepack.h"
#include <nlohmann/json.hpp>

#define POINT_MESSAGE_SIZE_BYTES 8

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
        ros::Publisher update_params_pub_;

        ros::Publisher serial_console_pub_;
        ros::Subscriber serial_console_sub_;

        uint64_t last_pt_cloud_time=0;


        // WebSocket server
        WsServer server;
        std::unique_ptr<std::thread> serverThread;
        std::promise<unsigned short> server_port;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;

        // Markers
        visualization_msgs::MarkerArray cup_marker;
        visualization_msgs::Marker trajectory;
        visualization_msgs::Marker launcher_target;
        void cup_markerCallback( const visualization_msgs::MarkerArray::ConstPtr& msg );
        void trajectoryCallback( const visualization_msgs::Marker::ConstPtr& msg );
        void launcher_targetCallback( const visualization_msgs::Marker::ConstPtr& msg );
        ros::Subscriber cup_marker_sub_;
        ros::Subscriber trajectory_sub_;
        ros::Subscriber launcher_target_sub_;

        //Table
        ros::Subscriber table_sub_;
        void tableCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg);
        geometry_msgs::PolygonStamped table_poly;

        // RPM and Yaw and velocity commands
        ros::Subscriber yaw_cmd_sub_;
        void yaw_cmdCallback(const std_msgs::Int8::ConstPtr& msg);
        std_msgs::Int8 yaw_cmd;

        ros::Subscriber rpm_cmd_sub_;
        void rpm_cmdCallback(const std_msgs::Float32::ConstPtr& msg);
        std_msgs::Float32 rpm_cmd;

        ros::Subscriber velocity_cmd_sub_;
        void velocity_cmdCallback(const std_msgs::Float32::ConstPtr& msg);
        std_msgs::Float32 velocity_cmd;

        ros::Subscriber pt_cloud_sub_;
        void pt_cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg);
        pcl::PointCloud<pcl::PointXYZRGB> pt_cloud;

        std::string pt_cloud_string_buffer;
        uint8_t *pt_cloud_array; 
        int pt_cloud_length;

        int tick_counter;

};

#endif
