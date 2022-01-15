#include "SocketTelemetryNode.h"

SocketTelemetryNode::SocketTelemetryNode( ros::NodeHandle nh )
{
    nh_ = nh;

    server.config.port = 8080;

    auto &endpoint = server.endpoint["^/data/?$"];

    endpoint.on_open = [](std::shared_ptr<WsServer::Connection> connection) {
        std::cout << "Server: Opened connection " << connection.get() << std::endl;
    };

    endpoint.on_close = [](std::shared_ptr<WsServer::Connection> connection, int status, const std::string &reason) {
        std::cout << "Server: Closed connection " << connection.get() << " with status code " << status << std::endl;
    };

    endpoint.on_error = [](std::shared_ptr<WsServer::Connection> connection, const SimpleWeb::error_code &ec) {
        std::cout << "Server: Error in connection " << connection.get() << ". "
            << "Error: " << ec << ", error message: " << ec.message() << std::endl;
    };

    endpoint.on_message = [&](std::shared_ptr<WsServer::Connection> connection, std::shared_ptr<WsServer::InMessage> message) {
        // Parse JSON command message
        auto jsonMsg = json::parse(message->string());
        handleCommand(jsonMsg);
    };

    trigger_pub_ = nh.advertise<std_msgs::Empty> ("/launcher/trigger", 1);
    rpm_cmd_pub_ = nh.advertise<std_msgs::Float32>( "/launcher/rpm_cmd", 1 );
    zero_gimbal_pub_ = nh.advertise<std_msgs::Empty> ("/launcher/reset", 1);
}

void SocketTelemetryNode::handleCommand(json& jsonMsg) {
    int type = jsonMsg["type"];
    std::string key = jsonMsg["key"];

    if (type == 0) {
        std::cout << "Got command '" << key << "'..." << std::endl;

        // Handle commands
        if (key == "shutdown") {
            system("sudo shutdown now");
        }
        if (key == "restart") {
            system("sudo shutdown -r now");
        }
        if (key == "restart_ros") {
            system("systemctl restart pongrobot.service");
        }
        if (key == "zero_yaw_gimbal") {
            std_msgs::Empty request;
            zero_gimbal_pub_.publish(request);
        }
        if (key == "launch_ball") {
            std_msgs::Empty request;
            trigger_pub_.publish(request);
        }
        if (key == "spin_up_motors") {
            std_msgs::Float32 request;
            request.data = 1000.0f;
            rpm_cmd_pub_.publish(request);
        }
    } else if (type == 1) {
        std::cout << "Got parameter update,key: '" << key << "', value: " << std::endl;
    }
}

void SocketTelemetryNode::start()
{
    ROS_INFO("[SocketTelemetryNode] Starting WebSocket server...");

    serverThread = std::make_unique<std::thread>([&]() {
        // Start server
        server.start([&](unsigned short port) {
            server_port.set_value(port);
            ROS_INFO("[SocketTelemetryNode] Started server on port %d", port);
        });
    });
}

void SocketTelemetryNode::update()
{
    // If we have updates to any of the data, send a message over the websocket

}

void SocketTelemetryNode::stop()
{
    ROS_INFO("[SocketTelemetryNode] Stopping WebSocket server...");

    server.stop();
    serverThread->join();
}
