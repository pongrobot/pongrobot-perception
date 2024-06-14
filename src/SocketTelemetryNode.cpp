#include "SocketTelemetryNode.h"

SocketTelemetryNode::SocketTelemetryNode( ros::NodeHandle nh ):tfListener(tfBuffer)
{
    nh_ = nh;

    server.config.port = 8081;

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
    update_params_pub_ = nh.advertise<std_msgs::Empty> ("/calibration/request",1);
    
    // Marker subscriptions
    cup_marker_sub_ = nh.subscribe<visualization_msgs::MarkerArray>("/detector/cup_marker", 1, &SocketTelemetryNode::cup_markerCallback, this);
    trajectory_sub_ = nh.subscribe<visualization_msgs::Marker>("/launcher/trajectory", 1, &SocketTelemetryNode::trajectoryCallback, this);
    launcher_target_sub_ = nh.subscribe<visualization_msgs::Marker>("/launcher/launcher_target", 1, &SocketTelemetryNode::launcher_targetCallback, this);
    
    table_sub_ = nh.subscribe<geometry_msgs::PolygonStamped>("/detector/table", 1, &SocketTelemetryNode::tableCallback, this);
    yaw_cmd_sub_ = nh.subscribe<std_msgs::Float3d>("/launcher/yaw_cmd", 1, &SocketTelemetryNode::yaw_cmdCallback, this);
    rpm_cmd_sub_ = nh.subscribe<std_msgs::Float32>("/launcher/rpm_cmd", 1, &SocketTelemetryNode::rpm_cmdCallback, this);
    velocity_cmd_sub_ = nh.subscribe<std_msgs::Float32>("/launcher/velocity_cmd", 1, &SocketTelemetryNode::velocity_cmdCallback, this);
    pt_cloud_sub_ = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>("/camera/depth/color/points", 1, &SocketTelemetryNode::pt_cloudCallback, this);

    
    pt_cloud_length = 640*480*POINT_MESSAGE_SIZE_BYTES*.5;
    pt_cloud_array = (uint8_t*)malloc(pt_cloud_length*sizeof(uint8_t));
    for (int i=0; i<pt_cloud_length;i++){
        pt_cloud_array[i]=0;
    } 
    
}
inline uint64_t getCurrentMillis() {
    return std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::system_clock::now().time_since_epoch()).count();
}
void SocketTelemetryNode::handleCommand(json& jsonMsg) {
    int type = jsonMsg["type"];
    std::string key = jsonMsg["key"];

    if (type == 0) {
        ROS_INFO("[SocketTelemetryNode] Got command: %s", key.c_str() );

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
            request.data = 2000.0f;
            rpm_cmd_pub_.publish(request);
        }
        if (key == "calibrate"){
            std_msgs::Empty request;
            update_params_pub_.publish(request);
        }
    } else if (type == 1) {
        float value = jsonMsg["value"];
        if (key=="max-depth")
        {
            nh_.setParam("/detector/filter/passthrough_max_depth",value);
        }
        if (key=="min-depth")
        {
            nh_.setParam("/detector/filter/passthrough_min_depth",value);
        }
        if (key=="max-height")
        {
            nh_.setParam("/detector/filter/object_max_height",value);
        }
        if (key=="vesc-fudge")
        {
            nh_.setParam("/launcher/vesc/fudge",value);
        }
        if (key=="eps-angle")
        {
            nh_.setParam("/detector/segment/eps_angle",value);
        }
        if (key=="distance-threshold")
        {
            nh_.setParam("/detector/segment/distance_threshold",value);
        }
        if (key=="cluster-tolerance")
        {
            nh_.setParam("/detector/cluster/tolerance",value);
        }
        if (key=="min-cluster-size")
        {
            nh_.setParam("/detector/cluster/min_cluster_size",value);
        }
        if (key=="max-cluster-size")
        {
            nh_.setParam("/detector/cluster/max_cluster_size",value);
        }
        
        // send empty message std messages to /dector/restart

        ROS_INFO("Got parameter update forkey: %s", key.c_str());
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
    tick_counter+=1;
    if (tick_counter%5 != 0){
        return;
    }
    // If we have updates to any of the data, send a message over the websocket
    // Getting all the parameters and sending them in a json object
    json j;
    j["max-depth"]=nullptr;
    j["min-depth"]=nullptr;
    j["max-height"]=nullptr;
    j["vesc-fudge"]=nullptr;
    j["eps-angle"]=nullptr;
    j["distance-threshold"]=nullptr;
    j["cluster-tolerance"]=nullptr;
    j["min-cluster-size"]=nullptr;
    j["max-cluster-size"]=nullptr;
    j["topic"]="parameters";
    float passthrough_max_depth;
    if (nh_.getParam("/detector/filter/passthrough_max_depth", passthrough_max_depth))
    {
        j["max-depth"]=passthrough_max_depth;
    }
    float passthrough_min_depth;
    if (nh_.getParam("/detector/filter/passthrough_min_depth", passthrough_min_depth))
    {
        j["min-depth"]=passthrough_min_depth;
    }
    float object_max_height;
    if (nh_.getParam("/detecter/filter/object_max_height", object_max_height))
    {
        j["max-height"]=object_max_height;
    }
    float fudge;
    if (nh_.getParam("/launcher/vesc/fudge", fudge))
    {
        j["vesc-fudge"]=fudge;
    }
    float eps_angle;
    if (nh_.getParam("/detector/segment/eps_angle", eps_angle))
    {
        j["eps-angle"]=eps_angle;
    }
    float distance_threshold;
    if (nh_.getParam("/detector/segment/distance_threshold", distance_threshold))
    {
        j["distance-threshold"]=distance_threshold;
    }
    float cluster_tolerance;
    if (nh_.getParam("/detector/cluster/cluster_tolerance", cluster_tolerance))
    {
        j["cluster-tolerance"]=cluster_tolerance;
    }
    float min_cluster_size;
    if (nh_.getParam("/detector/cluster/min_cluster_size", min_cluster_size))
    {
        j["min-cluster-size"]=min_cluster_size;
    }
    float max_cluster_size;
    if (nh_.getParam("/detector/cluster/max_cluster_size", max_cluster_size))
    {
        j["max-cluster-size"]=max_cluster_size;
    }


    // Getting the transforms and sending them in a json object j2
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform("world", "camera_link",ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    json j2;
    j2["topic"]="camera_frame";

    j2["x"]=transformStamped.transform.translation.x;
    j2["y"]=transformStamped.transform.translation.y;
    j2["z"]=transformStamped.transform.translation.z;
    j2["rx"]=transformStamped.transform.rotation.x;
    j2["ry"]=transformStamped.transform.rotation.y;
    j2["rz"]=transformStamped.transform.rotation.z; // swapping y and z for the ros transform to 3js. Nevermind

    // Cup Marker
    json j3;
    j3["topic"]="cup_marker";
    j3["markers"]=json::array();
    for (int i=0; i<cup_marker.markers.size(); i++)
    {
        json j4;
        j4["x"]=cup_marker.markers[i].pose.position.x;
        j4["y"]=cup_marker.markers[i].pose.position.y;
        j4["z"]=cup_marker.markers[i].pose.position.z;
        j3["markers"].push_back(j4);
    }
    
    // Table
    json j_table;
    j_table["topic"]="table_marker";
    j_table["vertices"]=json::array();
    for (int i=0; i<table_poly.polygon.points.size(); i++)
    {
        json j4;
        j4["x"]=table_poly.polygon.points[i].x;
        j4["y"]=table_poly.polygon.points[i].y;
        j4["z"]=table_poly.polygon.points[i].z;
        j_table["vertices"].push_back(j4);
    }

    json j_trajectory;
    j_trajectory["topic"]="launcher_trajectory";
    j_trajectory["points"]=json::array();
    for (int i=0; i<trajectory.points.size(); i++)
    {
        json j4;
        j4["x"]=trajectory.points[i].x;
        j4["y"]=trajectory.points[i].y;
        j4["z"]=trajectory.points[i].z;
        j_trajectory["vertices"].push_back(j4);
    }

    //Launcher target, rpm, velocity, yaw
    json j_target;
    j_target["topic"]="launcher_target";
    j_target["x"]=launcher_target.pose.position.x;
    j_target["y"]=launcher_target.pose.position.y;
    j_target["z"]=launcher_target.pose.position.z;
    
    json j_rpm;
    j_rpm["topic"]="launcher_rpm";
    j_rpm["rpm"] = rpm_cmd.data;

    json j_velocity;
    j_velocity["topic"]="launcher_velocity";
    j_velocity["velocity"] = velocity_cmd.data;
    // ROS_WARN("%s",j_velocity.dump().c_str());

    json j_yaw;
    j_yaw["topic"]="launcher_yaw";
    j_yaw["yaw"] = yaw_cmd.data;

    for (auto& connection : server.get_connections()) {
        connection->send(j.dump());
        connection->send(j2.dump());
        connection->send(j3.dump());
        connection->send(j_table.dump());
        connection->send(j_target.dump());
        connection->send(j_yaw.dump());
        connection->send(j_rpm.dump());
        connection->send(j_velocity.dump());
        connection->send(j_trajectory.dump()); 

    }
    // ROS_WARN("%s",j.dump().c_str());
    
}

void SocketTelemetryNode::stop()
{
    ROS_INFO("[SocketTelemetryNode] Stopping WebSocket server...");

    server.stop();
    serverThread->join();
}

void SocketTelemetryNode::cup_markerCallback( const visualization_msgs::MarkerArray::ConstPtr& msg )
{
    cup_marker = *msg;
}
void SocketTelemetryNode::trajectoryCallback( const visualization_msgs::Marker::ConstPtr& msg )
{
    trajectory = *msg;
}
void SocketTelemetryNode::launcher_targetCallback( const visualization_msgs::Marker::ConstPtr& msg )
{
    launcher_target = *msg;
}
void SocketTelemetryNode::tableCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
    table_poly = *msg;
}
void SocketTelemetryNode::yaw_cmdCallback(const std_msgs::Float32::ConstPtr& msg)
{
    yaw_cmd = *msg;
}
void SocketTelemetryNode::rpm_cmdCallback(const std_msgs::Float32::ConstPtr& msg)
{
    rpm_cmd = *msg;
}
void SocketTelemetryNode::velocity_cmdCallback(const std_msgs::Float32::ConstPtr& msg)
{
    velocity_cmd = *msg;
}
void SocketTelemetryNode::pt_cloudCallback( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg )
{
    // uint64_t current_time=getCurrentMillis();
    if (tick_counter%5==0)
    {
        for(int nIndex = 0; nIndex < msg->points.size(); nIndex ++)
        {
            if (nIndex%2==0)
            {
                /*
                Packed point:
                - 1 unsigned byte for R
                - 1 unsigned byte for G
                - 1 unsigned byte for B
                - 2 bytes for X 
                - 2 bytes for Y
                - 2 bytes for Z
                */
                int array_ind = nIndex*POINT_MESSAGE_SIZE_BYTES*.5;

                float x = msg->points[nIndex].x;
                float y = msg->points[nIndex].y;
                float z = msg->points[nIndex].z;

                //unsigned char const * x_bytes = reinterpret_cast<unsigned char const *>(&(msg->points[nIndex].x));
                //unsigned char const * y_bytes = reinterpret_cast<unsigned char const *>(&(msg->points[nIndex].y));
                //unsigned char const * z_bytes = reinterpret_cast<unsigned char const *>(&(msg->points[nIndex].z));

                //pt_cloud_array[array_ind+0]=msg->points[nIndex].r;
                //pt_cloud_array[array_ind+1]=msg->points[nIndex].g;
                //pt_cloud_array[array_ind+2]=msg->points[nIndex].b;

                uint16_t xScaled = 0;
                uint16_t yScaled = 0;
                uint16_t zScaled = 0;

                bool isValidCoord = true;
                if (x < -5.0 || x > 5.0) {
                    isValidCoord = false;
                }
                if (y < -5.0 || y > 5.0) {
                    isValidCoord = false;
                }
                if (z < 0.0 || z > 10.0) {
                    isValidCoord = false;
                }
                if (isValidCoord) {
                    xScaled = (uint16_t)(((x + 5.0) / 10.0) * 65535.0);
                    yScaled = (uint16_t)(((y + 5.0) / 10.0) * 65535.0);
                    zScaled = (uint16_t)(((z + 0.0) / 10.0) * 65535.0);
                }
                unsigned char const * x_bytes = reinterpret_cast<unsigned char const *>(&xScaled);
                unsigned char const * y_bytes = reinterpret_cast<unsigned char const *>(&yScaled);
                unsigned char const * z_bytes = reinterpret_cast<unsigned char const *>(&zScaled);

                pt_cloud_array[array_ind+0]=x_bytes[0];
                pt_cloud_array[array_ind+1]=x_bytes[1];
                pt_cloud_array[array_ind+2]=y_bytes[0];
                pt_cloud_array[array_ind+3]=y_bytes[1];
                pt_cloud_array[array_ind+4]=z_bytes[0];
                pt_cloud_array[array_ind+5]=z_bytes[1];

                /*
                pt_cloud_array[array_ind+0]=x_bytes[0];
                pt_cloud_array[array_ind+1]=x_bytes[1];
                pt_cloud_array[array_ind+2]=x_bytes[2];
                pt_cloud_array[array_ind+3]=x_bytes[3];

                pt_cloud_array[array_ind+4]=y_bytes[0];
                pt_cloud_array[array_ind+5]=y_bytes[1];
                pt_cloud_array[array_ind+6]=y_bytes[2];
                pt_cloud_array[array_ind+7]=y_bytes[3];

                pt_cloud_array[array_ind+8]=z_bytes[0];
                pt_cloud_array[array_ind+9]=z_bytes[1];
                pt_cloud_array[array_ind+10]=z_bytes[2];
                pt_cloud_array[array_ind+11]=z_bytes[3];
                */

                
                uint8_t red   = (msg->points[nIndex].r * 8) / 256;
                uint8_t green = (msg->points[nIndex].g * 8) / 256;
                uint8_t blue  = (msg->points[nIndex].b * 4) / 256;
                pt_cloud_array[array_ind+6] = (red << 5) | (green << 2) | blue;
            }
        }
        pt_cloud_string_buffer.assign((char*)pt_cloud_array, pt_cloud_length * sizeof(uint8_t));
        for (auto& connection : server.get_connections()) {
            connection->send(pt_cloud_string_buffer,[](const SimpleWeb::error_code& ec){}, 130);
        }  
    }
}

