#include "CupDetector.h"

CupDetector::
CupDetector( ros::NodeHandle nh ):
    tf_listener_(tf_buffer_),
    restart_(false),
    calibrate_(false),
    state_(DetectorState::RUNNING) 
{
    nh_ = nh;
    calibration_time_ = ros::Time::now();

    // Read in detector parameters
    load_params();        

    // Init Publishers and Subscribers
    cloud_sub_ = nh_.subscribe<pcl::PointCloud<pcl::PointXYZRGB>> ("/camera/depth/color/points", 1, &CupDetector::pointCloudCallback, this);
    restart_sub_ = nh_.subscribe<std_msgs::Empty> ("/detector/restart", 1, &CupDetector::restartCallback, this);
    calibrate_sub_ = nh_.subscribe<std_msgs::Empty> ("/detector/calibrate", 1, &CupDetector::calibrateCallback, this);
    state_pub_ = nh.advertise<std_msgs::Int8>( "/detector/state", 1 );
    if ( publish_obj_cloud_ ) { obj_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("/detector/obj", 1); }
    if ( publish_cluster_cloud_ ) { cluster_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("/detector/cluster", 1); }
    if ( publish_cup_markers_ ) { marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>( "/detector/cup_marker", 1 ); };
    cup_pose_pub_ = nh.advertise<geometry_msgs::PoseArray>( "/detector/cup_array", 1 );
    
    // Init clouds
    cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() ); 
    obj_cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() ); 
    cluster_cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() ); 

    // Init colorwheel
    colorwheel_.push_back( Eigen::Vector3i(255, 0,     0) );
    colorwheel_.push_back( Eigen::Vector3i(255, 125,   0) );
    colorwheel_.push_back( Eigen::Vector3i(255, 255,   0) );
    colorwheel_.push_back( Eigen::Vector3i(125, 255,   0) );
    colorwheel_.push_back( Eigen::Vector3i(  0, 255,   0) );
    colorwheel_.push_back( Eigen::Vector3i(  0, 255, 125) );
    colorwheel_.push_back( Eigen::Vector3i(  0, 255, 255) );
    colorwheel_.push_back( Eigen::Vector3i(  0, 125, 255) );
    colorwheel_.push_back( Eigen::Vector3i(  0,   0, 255) );
    colorwheel_.push_back( Eigen::Vector3i(125,   0, 255) );
    colorwheel_.push_back( Eigen::Vector3i(255,   0, 255) );
    colorwheel_.push_back( Eigen::Vector3i(255,   0, 125) );
}

void
CupDetector::
pointCloudCallback( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg )
{
    try
    {
        // wait for the correct transform for up to 5 sec (supress startup errors)
        tf_buffer_.lookupTransform(target_frame_id_, msg->header.frame_id, ros::Time::now(), ros::Duration(5.0));
        pcl_ros::transformPointCloud(target_frame_id_, *msg, *cloud_, tf_buffer_);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
    } 
}

void
CupDetector::
restartCallback(const std_msgs::Empty::ConstPtr& msg)
{
    restart_ = true;
}

void 
CupDetector::
calibrateCallback(const std_msgs::Empty::ConstPtr& msg)
{
    calibrate_ = true;
}

void
CupDetector::
load_params()
{
    if ( !nh_.getParam("target_frame_id", target_frame_id_) )
    {
        ROS_ERROR("CupDetector cannot load param: %s/target_frame_id", nh_.getNamespace().c_str() );
    }

    if ( !nh_.getParam("filter/object_max_height", obj_max_height_) )
    {
        ROS_ERROR("CupDetector cannot load param: %s/filter/object_max_height", nh_.getNamespace().c_str() );
    }

    if ( !nh_.getParam("cluster/tolerance", cluster_tolerance_) )
    {
        ROS_ERROR("CupDetector cannot load param: %s/cluster/tolerance", nh_.getNamespace().c_str() );
    }

    if ( !nh_.getParam("cluster/min_cluster_size", min_cluster_size_) )
    {
        ROS_ERROR("CupDetector cannot load param: %s/cluster/min_cluster_size", nh_.getNamespace().c_str() );
    }

    if ( !nh_.getParam("cluster/max_cluster_size", max_cluster_size_) )
    {
        ROS_ERROR("CupDetector cannot load param: %s/cluster/max_cluster_size", nh_.getNamespace().c_str() );
    }

    if ( !nh_.getParam("debug/publish_obj_cloud", publish_obj_cloud_) )  
    {
        ROS_ERROR("CupDetector cannot load param: %s/debug/publish_obj_cloud", nh_.getNamespace().c_str() );
    }

    if ( !nh_.getParam("debug/publish_cluster_cloud", publish_cluster_cloud_) )
    {
        ROS_ERROR("CupDetector cannot load param: %s/debug/publish_cluster_cloud", nh_.getNamespace().c_str() );
    }

    if ( !nh_.getParam("debug/publish_cup_markers", publish_cup_markers_) )
    {
        ROS_ERROR("CupDetector cannot load param: %s/debug/publish_cup_markers", nh_.getNamespace().c_str() );
    }

    if ( !nh_.getParam("/cluster_splitting/enabled", cluster_splitting_) )
    {
        ROS_ERROR("CupDetector cannot load param: /cluster_splitting/enabled");
    }

    if ( !nh_.getParam("/cluster_splitting/max_width_m", max_cluster_width_) )
    {
        ROS_ERROR("CupDetector cannot load param: /cluster_splitting/max_width_m");
    }


    double calibration_timeout_val;
    if ( !nh_.getParam("/game/calibration_timeout", calibration_timeout_val) )
    {
        calibration_timeout_val = 60.0; // Set default value
        ROS_WARN("CupDetector cannot load param: /game/calibration_timout. Using default value of 30sec");
    }
    calibration_timeout_ = ros::Duration(calibration_timeout_val+30);
}

void
CupDetector::
run()
{
    switch(state_)
    {
        case DetectorState::CALIBRATING:
        {

            // Check for restart
            if ( restart_ )
            {
                state_ = DetectorState::RESTARTING;
                ROS_INFO("[CupDetector] RUNNING->RESTARTING: Received restart request");
                restart_ = false;
            }

            // Check for timeout
            if ( ros::Time::now() - calibration_time_ > calibration_timeout_ )
            {
                state_ = DetectorState::RESTARTING;
                ROS_INFO("[CupDetector] RUNNING->RESTARTING: Calibration timeout");
            }

            break;
        }
        case DetectorState::RESTARTING:
        {
            
            // Reload new parameters from the server
            load_params();

            state_ = DetectorState::RUNNING;
            ROS_INFO("[CupDetector] RESTARTING->RUNNING: Reloading parameters");
            break;
        }
        case DetectorState::RUNNING:
        {
            if ( calibrate_ )
            {
                state_ = DetectorState::CALIBRATING;
                calibration_time_ = ros::Time::now();
                ROS_INFO("[CupDetector] RUNNING->CALIBRATING: Received calibrate request");
                calibrate_ = false;
                restart_ = false;
            }

            if ( restart_ )
            {
                state_ = DetectorState::RESTARTING;
                ROS_INFO("[CupDetector] RUNNING->RESTARTING: Received restart request");
                restart_ = false;
            }

            // Actually run the detection
            detect();

            break;
        }
    }

    std_msgs::Int8 state_msg;
    state_msg.data = state_;
    state_pub_.publish(state_msg);
}

void
CupDetector::
detect()
{
    // Check for calibration paramters 
    std::vector<float> minPt;
    nh_.getParam("calibration/table_plane/min", minPt);
    std::vector<float> maxPt;
    nh_.getParam("calibration/table_plane/max", maxPt);

    if ((!cloud_->empty()) && minPt.size() == 3 && maxPt.size() == 3)
    { 
        // Pull out segmented objects on table
        pcl::CropBox<pcl::PointXYZRGB> box_filter;
        box_filter.setMin(Eigen::Vector4f(minPt[0], minPt[1], maxPt[2], 1.0));
        box_filter.setMax(Eigen::Vector4f(maxPt[0], maxPt[1], maxPt[2] + obj_max_height_, 1.0));
        box_filter.setInputCloud(cloud_);
        box_filter.filter(*obj_cloud_);
       
        if ( publish_obj_cloud_ )
        { 
            obj_cloud_pub_.publish(obj_cloud_);
        }

        if ( obj_cloud_->size() > 0 )
        {
            // Cluster the objects to find targets
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
            tree->setInputCloud(obj_cloud_);

            // Run clustering
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
            ec.setClusterTolerance(cluster_tolerance_);
            ec.setMinClusterSize(min_cluster_size_);
            ec.setMaxClusterSize(max_cluster_size_);
            ec.setSearchMethod(tree);
            ec.setInputCloud(obj_cloud_);
            ec.extract(cluster_indices);

            // Split clusters
            if (cluster_splitting_)
            {
                // split in x, then in y
                split_clusters(obj_cloud_, cluster_indices, 0);
                split_clusters(obj_cloud_, cluster_indices, 1);
            }


            // Setup visualization markers
            visualization_msgs::MarkerArray cup_markers;
            visualization_msgs::Marker clear_markers;
            clear_markers.header.frame_id = target_frame_id_;    
            clear_markers.action = visualization_msgs::Marker::DELETEALL;
            cup_markers.markers.push_back(clear_markers);

            // Cup pose output
            geometry_msgs::PoseArray cup_pose_array;
            cup_pose_array.header.frame_id = target_frame_id_;
            cup_pose_array.header.stamp = ros::Time();

            // Iterate through clusters
            int cup_index =0;
            cluster_cloud_ = obj_cloud_->makeShared();
            cluster_cloud_->clear();

            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            {
                if ( publish_cluster_cloud_ )
                {
                    // Build cluster visualization
                    for (const auto& idx : it->indices)
                    {
                        pcl::PointXYZRGB point = (*obj_cloud_)[idx];
                        Eigen::Vector3i color = getColor( cup_index );
                        point.r = color[0];
                        point.g = color[1];
                        point.b = color[2];
                        
                        cluster_cloud_->push_back( point );
                    }
                }
                // Get centroid of cup
                Eigen::Vector4f centroid;
                if ( pcl::compute3DCentroid(*obj_cloud_, it->indices, centroid) )
                {
                    if ( publish_cup_markers_ )
                    {
                        cup_markers.markers.push_back( buildCupMarker(cup_index, centroid) );
                    }

                    pcl::PointXYZRGB minPt, maxPt;
                    pcl::getMinMax3D (*cluster_cloud_, minPt, maxPt);

                    // create Pose
                    geometry_msgs::Pose cup_pose;
                    cup_pose.position.x = centroid[0];
                    cup_pose.position.y = centroid[1];
                    cup_pose.position.z = maxPt.z;
                    cup_pose_array.poses.push_back(cup_pose);

                    cup_index ++;
                }
                
            }

            // Publish Cup data
            if ( cup_markers.markers.size() > 0 ) 
            {
                if ( publish_cup_markers_ )
                {
                    marker_pub_.publish(cup_markers); 
                }
                if ( publish_cluster_cloud_ )
                {
                    cluster_cloud_pub_.publish(cluster_cloud_);
                }

                // Output cup positions
                cup_pose_pub_.publish(cup_pose_array);
            }
        }
    }
}

visualization_msgs::Marker
CupDetector::
buildCupMarker( int cup_index, Eigen::Vector4f centroid )
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = target_frame_id_;
    marker.header.stamp = ros::Time();
    marker.ns = "cups";
    marker.id = cup_index;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = centroid[0];
    marker.pose.position.y = centroid[1];
    marker.pose.position.z = centroid[2];
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 0.8;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    return marker;
}

Eigen::Vector3i
CupDetector::
getColor( int seed )
{
    // generate color from the colorwheel
    if (colorwheel_.size() > 0)
    {
        return colorwheel_[ seed % (int) colorwheel_.size() ]; 
    }
    else
    {
        ROS_WARN("Colorwheel uninitialized, returning RED");
        return Eigen::Vector3i( 255, 0, 0);
    }
}

void CupDetector::split_clusters(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    std::vector<pcl::PointIndices>& clusters,
    const int& dim)
{
    // input dimension validation
    if (dim < 0 || dim > 2)
    {
        ROS_ERROR("Failed to split clusters along invalid dimenstion %d. Valid dimentions: [0, 1, 2]", dim); 
        return;
    }

    try
    {
        auto split_clusters = std::make_shared<std::vector<pcl::PointIndices>>();
        for (auto& cluster : clusters)
        {
            // find cluster extent
            Eigen::Vector4f min_pt, max_pt, extent ;
            pcl::getMinMax3D(*cloud, cluster, min_pt, max_pt);
            extent = max_pt - min_pt;

            // calculate number of splits
            int num_subclusters = std::round(extent[dim]/max_cluster_width_);
            if (num_subclusters <= 1)
            {
                // no splits needed
                split_clusters->push_back(cluster);
            }
            else
            {
                // split into subclusters 
                float subcluster_delta = extent[dim] / num_subclusters;
                for(int i = 0; i < num_subclusters; i++)
                {
                    // find subcluster bounds
                    float subcluster_min = min_pt[dim] + i * subcluster_delta;
                    float subcluster_max = min_pt[dim] + (i+1) * subcluster_delta;

                    // find subcluster indices
                    pcl::PointIndices subcluster;
                    for (const auto& point_idx : cluster.indices)
                    {
                        float point_val;
                        if (dim == 0)
                            point_val = (*cloud)[point_idx].x;
                        else if (dim == 1)
                            point_val = (*cloud)[point_idx].y;
                        else if (dim == 2)
                            point_val = (*cloud)[point_idx].z;

                        if (subcluster_min <= point_val && point_val <= subcluster_max )
                            subcluster.indices.push_back(point_idx);
                    }
                    
                    if (subcluster.indices.size() >= min_cluster_size_)
                        split_clusters->emplace_back(std::move(subcluster));
                }
            }
        }

        clusters.clear();
        for (int i = 0; i < split_clusters.size(); i++) {
            clusters.push_back(split_clusters[i]);
        }
        //clusters = split_clusters;
    }
    catch(const std::exception& e)
    {
        ROS_WARN("Failed to load clustering splitting params: %s", e.what());
    }
}