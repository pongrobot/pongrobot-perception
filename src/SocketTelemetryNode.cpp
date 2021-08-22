#include "SocketTelemetryNode.h"

SocketTelemetryNode::SocketTelemetryNode( ros::NodeHandle nh )
{
    nh_ = nh;

    server.config.port = 8080;

    auto &echo = server.endpoint["^/data/?$"];

    echo.on_open = [](std::shared_ptr<WsServer::Connection> connection) {
        std::cout << "Server: Opened connection " << connection.get() << std::endl;
    };

    echo.on_close = [](std::shared_ptr<WsServer::Connection> connection, int status, const std::string &reason) {
        std::cout << "Server: Closed connection " << connection.get() << " with status code " << status << std::endl;
    };

    echo.on_error = [](std::shared_ptr<WsServer::Connection> connection, const SimpleWeb::error_code &ec) {
        std::cout << "Server: Error in connection " << connection.get() << ". "
            << "Error: " << ec << ", error message: " << ec.message() << std::endl;
    };
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
}

void SocketTelemetryNode::stop()
{
    ROS_INFO("[SocketTelemetryNode] Stopping WebSocket server...");

    server.stop();
    serverThread->join();
}