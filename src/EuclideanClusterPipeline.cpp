#include "EuclideanClusterPipeline.h"

EuclideanClusterPipeline::
EuclideanClusterPipeline( ros::NodeHandle nh, std::string frame_id ):
    frame_id_(frame_id),
    tf_listener_(tf_buffer_)
{
    nh_ = nh;
    loadParams();

    // Init Publishers and Subscribers
    camera_cloud_sub_ = nh_.subscribe<pcl::PointCloud<pcl::PointXYZRGB>> ("/camera/depth/color/points", 1, &EuclideanClusterPipeline::pointCloudCallback, this);
    cup_pose_pub_ = nh.advertise<visualization_msgs::MarkerArray>( "output", 1 );
    if ( debug_flags_.publish_filtered_cloud ) { obj_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("filtered", 1); }
    if ( debug_flags_.publish_cluster_cloud ) { cluster_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("cluster", 1); }
    if ( debug_flags_.publish_cup_markers ) { marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>( "cup_marker", 1 ); };
    

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
EuclideanClusterPipeline::
run()
{
    loadParams();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pipeline_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& cloud : cloud_frames_)
    {
        *pipeline_cloud+=*cloud;
    }

    if( !pipeline_cloud->empty() )
    {
        cup_pose_pub_.publish(process(pipeline_cloud));
    }
}


geometry_msgs::PoseArray
EuclideanClusterPipeline::
process( pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud )
{
    // Cup pose output
    geometry_msgs::PoseArray cup_pose_array;
    cup_pose_array.header.frame_id = frame_id_;
    cup_pose_array.header.stamp = ros::Time();

    // Setup visualization
    visualization_msgs::MarkerArray cup_markers;
    visualization_msgs::Marker clear_markers;
    clear_markers.header.frame_id = frame_id_;    
    clear_markers.action = visualization_msgs::Marker::DELETEALL;
    cup_markers.markers.push_back(clear_markers);

    // Initialize new clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_viz_cloud;

    // Read calibration parameters
    std::vector<float> minPt, maxPt;
    nh_.getParam("/table/min", minPt);
    nh_.getParam("/table/max", maxPt);

    if ( ( !raw_cloud->empty() ) && 
         ( minPt.size() == 3 && maxPt.size() == 3)  )
    {
        //////////////////////////////////////////////////
        // Run filtering step
        //////////////////////////////////////////////////
        pcl::CropBox<pcl::PointXYZRGB> box_filter;
        box_filter.setMin(Eigen::Vector4f(minPt[0], minPt[1], maxPt[2], 1.0));
        box_filter.setMax(Eigen::Vector4f(maxPt[0], maxPt[1], maxPt[2] + crop_box_params_.max_object_height, 1.0));
        box_filter.setInputCloud(raw_cloud);
        box_filter.filter(*cloud);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud( cloud );
        sor.setMeanK(sor_params_.mean_k);
        sor.setStddevMulThresh(sor_params_.std_dev_mul_thresh);
        sor.filter(*cloud);

        pcl::VoxelGrid<pcl::PointXYZRGB> vgf;
        vgf.setInputCloud(cloud);
        vgf.setLeafSize(vgf_params_.leaf_size_x,
                        vgf_params_.leaf_size_y,
                        vgf_params_.leaf_size_z);
        vgf.filter(*cloud);

        // Write debug cloud
        if ( debug_flags_.publish_filtered_cloud )
        {
            obj_cloud_pub_.publish(cloud);
        }

        //////////////////////////////////////////////////
        // Run clustering algorithm
        //////////////////////////////////////////////////
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZRGB> );
        tree->setInputCloud( cloud );

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(ece_params_.cluster_tolerance);
        ec.setMinClusterSize(ece_params_.min_cluster_size);
        ec.setMaxClusterSize(ece_params_.max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        //////////////////////////////////////////////////
        // Run clustering splitting algorithm
        //////////////////////////////////////////////////
        int cup_index =0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            
            float x_max = std::numeric_limits<float>::min();
            float x_min = std::numeric_limits<float>::max();

            for (const auto& idx : it->indices)
            {
                pcl::PointXYZRGB point = (*cloud)[idx];
                x_max = std::max(x_max, point.x);
                x_min = std::min(x_min, point.x);
            }
 
            float x_extent = x_max - x_min;
            int num_cups = ( x_extent > ece_params_.max_cluster_width ) ? floor(x_extent/0.10) : 1;
            std::vector<pcl::PointIndices> cup_indices(num_cups);

            if ( num_cups > 1 )
            {
                // Build up index sets for each new sub-cluster
                for (const auto& idx : it->indices)
                {
                    pcl::PointXYZRGB point = (*cloud)[idx];
                    int subcluster_index = std::round( ( point.x - x_min ) / x_extent );
                    cup_indices[subcluster_index].indices.push_back(idx);
                }
            }
            else
            {
                // Use full set of indices
                cup_indices[0].indices = it->indices;
            }

            //////////////////////////////////////////////////
            // Process each cluster
            //////////////////////////////////////////////////
            for ( auto & indices : cup_indices )
            {
                // Color the cloud and append
                if ( debug_flags_.publish_cluster_cloud )
                {
                    // Loop through the cloud and color
                    for (const auto& idx : indices.indices)
                    {
                        pcl::PointXYZRGB point = (*cluster_viz_cloud)[idx];
                        Eigen::Vector3i color = getColor( cup_index );
                        point.r = color[0];
                        point.g = color[1];
                        point.b = color[2];
                        
                        cluster_viz_cloud->push_back( point );
                    }
                }

                // Calculate the cup pose
                Eigen::Vector4f centroid;
                if ( pcl::compute3DCentroid(*cloud, indices, centroid) )
                {
                    pcl::PointXYZRGB minPt, maxPt;
                    pcl::getMinMax3D (*cloud, minPt, maxPt);

                    geometry_msgs::Pose cup_pose;
                    cup_pose.position.x = centroid[0];
                    cup_pose.position.y = centroid[1];
                    cup_pose.position.z = maxPt.z;
                    cup_pose_array.poses.push_back(cup_pose);
                    
                    // Create marker and append
                    if ( debug_flags_.publish_cup_markers )
                    {
                        cup_markers.markers.push_back( buildCupMarker(cup_index, centroid) );
                    }

                    cup_index++;
                }
            }
        }

        //////////////////////////////////////////////////
        // Publish debug info
        //////////////////////////////////////////////////
        if ( debug_flags_.publish_cluster_cloud )
        {
            cluster_cloud_pub_.publish(cluster_viz_cloud);
        }
        if ( debug_flags_.publish_cup_markers )
        {
            marker_pub_.publish(cup_markers);
        }
    }

    return cup_pose_array;
}

/////////////////////////////////////////////////////////
// Callback
/////////////////////////////////////////////////////////

void
EuclideanClusterPipeline::
pointCloudCallback( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg )
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr incoming_cloud;

    try
    {
        // wait for the correct transform for up to 5 sec (surpress startup errors)
        tf_buffer_.lookupTransform(frame_id_, msg->header.frame_id, ros::Time::now(), ros::Duration(5.0));
        pcl_ros::transformPointCloud(frame_id_, *msg, *incoming_cloud, tf_buffer_);
        cloud_frames_.push_front(incoming_cloud);

        // Flush out old frames
        while ( cloud_frames_.size() > pipeline_params_.frames_to_keep )
        {
            cloud_frames_.pop_back();
        }
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s: Could not find transform to \"%s\", rejecting pointcloud.", ros::this_node::getName().c_str(), frame_id_.c_str());
    }
}

/////////////////////////////////////////////////////////
// Utility Functions
/////////////////////////////////////////////////////////

void
EuclideanClusterPipeline::
loadParams()
{
    // Load Pipeline Params
    if ( !nh_.getParam("pipeline/frames_to_keep", pipeline_params_.frames_to_keep) )
    {
        ROS_ERROR("Euclidean Cluster Pipeline cannot load param: %s/pipeline/frames_to_keep", nh_.getNamespace().c_str() );
    }

    // Load Crop Box Params
    if ( !nh_.getParam("crop_box/max_object_height", crop_box_params_.max_object_height) )
    {
        ROS_ERROR("Euclidean Cluster Pipeline cannot load param: %s/crop_box/max_object_height", nh_.getNamespace().c_str() );
    }

    // Load SOR params
    if ( !nh_.getParam("sor/mean_k", sor_params_.mean_k) )
    {
        ROS_ERROR("Euclidean Cluster Pipeline cannot load param: %s/sor/mean_k", nh_.getNamespace().c_str() );
    }
    if ( !nh_.getParam("sor/std_dev_mul_thresh", sor_params_.std_dev_mul_thresh) )
    {
        ROS_ERROR("Euclidean Cluster Pipeline cannot load param: %s/sor/std_dev_mul_thresh", nh_.getNamespace().c_str() );
    }

    // Load VGF params 
    if ( !nh_.getParam("vgf/leaf_size_x", vgf_params_.leaf_size_x) )
    {
        ROS_ERROR("Euclidean Cluster Pipeline cannot load param: %s/vgf/leaf_size_x", nh_.getNamespace().c_str() );
    }
    if ( !nh_.getParam("vgf/leaf_size_y", vgf_params_.leaf_size_y) )
    {
        ROS_ERROR("Euclidean Cluster Pipeline cannot load param: %s/vgf/leaf_size_y", nh_.getNamespace().c_str() );
    }
    if ( !nh_.getParam("vgf/leaf_size_z", vgf_params_.leaf_size_z) )
    {
        ROS_ERROR("Euclidean Cluster Pipeline cannot load param: %s/vgf/leaf_size_z", nh_.getNamespace().c_str() );
    }

    // Load ECE params
    if ( !nh_.getParam("ece/cluster_tolerance", ece_params_.cluster_tolerance) )
    {
        ROS_ERROR("Euclidean Cluster Pipeline cannot load param: %s/vgf/cluster_tolerance", nh_.getNamespace().c_str() );
    }
    if ( !nh_.getParam("ece/min_cluster_size", ece_params_.min_cluster_size) )
    {
        ROS_ERROR("Euclidean Cluster Pipeline cannot load param: %s/vgf/min_cluster_size", nh_.getNamespace().c_str() );
    }
    if ( !nh_.getParam("ece/max_cluster_size", ece_params_.max_cluster_size) )
    {
        ROS_ERROR("Euclidean Cluster Pipeline cannot load param: %s/vgf/max_cluster_size", nh_.getNamespace().c_str() );
    }
    if ( !nh_.getParam("ece/max_cluster_width", ece_params_.max_cluster_width) )
    {
        ROS_ERROR("Euclidean Cluster Pipeline cannot load param: %s/vgf/max_cluster_width", nh_.getNamespace().c_str() );
    }

    // Load debug params
    if ( !nh_.getParam("debug/publish_filtered_cloud", debug_flags_.publish_filtered_cloud) )
    {
        ROS_ERROR("Euclidean Cluster Pipeline cannot load param: %s/debug/publish_filtered_cloud", nh_.getNamespace().c_str() );
    }
    if ( !nh_.getParam("debug/publish_cluster_cloud", debug_flags_.publish_cluster_cloud) )
    {
        ROS_ERROR("Euclidean Cluster Pipeline cannot load param: %s/debug/publish_cluster_cloud", nh_.getNamespace().c_str() );
    }
    if ( !nh_.getParam("debug/publish_cup_markers", debug_flags_.publish_cup_markers) )
    {
        ROS_ERROR("Euclidean Cluster Pipeline cannot load param: %s/debug/publish_cup_markers", nh_.getNamespace().c_str() );
    }
}


Eigen::Vector3i
EuclideanClusterPipeline::
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

visualization_msgs::Marker
EuclideanClusterPipeline::
buildCupMarker( int cup_index, Eigen::Vector4f centroid )
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
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

/////////////////////////////////////////////////////////
// Main
/////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "euclidean_cluster_pipeline");
    ros::NodeHandle nh;

    // Load Params
    float loop_frq;
    std::string frame_id;
    if ( !nh.getParam("rate", loop_frq) )
    {
        ROS_ERROR("Euclidean Cluster Pipeline cannot load param: %s/rate", nh.getNamespace().c_str());
    }
    if ( !nh.getParam("frame_id", frame_id) )
    {
        ROS_ERROR("Euclidean Cluster Pipeline cannot load param: %s/frame_id", nh.getNamespace().c_str());
    }

    ros::Rate loop_rate(loop_frq);
    EuclideanClusterPipeline euclidean_cluster_pipeline(nh, frame_id);

    while (ros::ok())
    {
        euclidean_cluster_pipeline.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}