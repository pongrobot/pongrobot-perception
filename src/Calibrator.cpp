#include "Calibrator.h"

Calibrator::
Calibrator( ros::NodeHandle nh ):
    tf_listener_(tf_buffer_),
    run_calibration_(false)
{
    nh_ = nh;
    // Read in calibrator parameters
    loadParams();

    // Publishers and subscribers
    calibrate_sub_ = nh_.subscribe<std_msgs::Empty> ("/detector/calibrate", 1, &Calibrator::calibrateCallback, this);
    cloud_sub_ = nh_.subscribe<pcl::PointCloud<pcl::PointXYZRGB>> ("/camera/depth/color/points", 1, &Calibrator::pointCloudCallback, this);
    calibrate_complete_pub_ = nh_.advertise<std_msgs::Empty> ("/calibration/complete", 1);
    table_poly_pub_ = nh.advertise<geometry_msgs::PolygonStamped> ("/detector/table", 1);
    if (publish_table_cloud_) { surface_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2> ("/detector/surface", 1); }
    if (publish_table_poly_) { table_poly_pub_ = nh.advertise<geometry_msgs::PolygonStamped> ("/detector/table", 1); }

    // Init clouds
    cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() ); 
    passthrough_cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() ); 
    marked_cloud_.reset( new pcl::PointCloud<pcl::PointXYZRGB>() ); 

}

void
Calibrator::
run()
{
  if (run_calibration_)
  {
      loadParams();
      calibration_ = calibrate();

      if (calibration_.size() > 0)
      {
          run_calibration_ = false;
          ROS_INFO ("Calibration Successful"); 
          
          // Send updated calibration to parameter server          
          publishCalibration();

          // Send completion signal
          std_msgs::Empty completed;
          calibrate_complete_pub_.publish(completed);
      }
      else
      {
          ROS_ERROR ("Calibration failed: Could not estimate a planar model for the given dataset.");
      }
  }

}

void
Calibrator::
calibrateCallback(const std_msgs::Empty::ConstPtr& msg)
{
    run_calibration_ = true;
}

void
Calibrator::
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
Calibrator::
loadParams()
{
    if ( !nh_.getParam("target_frame_id", target_frame_id_) )
    {
        ROS_ERROR("CupDetector cannot load param: %s/target_frame_id", nh_.getNamespace().c_str() );
    }

    if ( !nh_.getParam("filter/passthrough_max_depth", passthrough_max_depth_) )
    {
        ROS_ERROR("CupDetector cannot load param: %s/filter/passthrough_max_depth", nh_.getNamespace().c_str() );
    }

    if ( !nh_.getParam("filter/passthrough_min_depth", passthrough_min_depth_) )
    {
        ROS_ERROR("CupDetector cannot load param: %s/filer/passthrough_min_depth", nh_.getNamespace().c_str() );
    }

    if ( !nh_.getParam("filter/object_max_height", obj_max_height_) )
    {
        ROS_ERROR("CupDetector cannot load param: %s/filter/object_max_height", nh_.getNamespace().c_str() );
    }

    if ( !nh_.getParam("segment/eps_angle", eps_angle_) )
    {
        ROS_ERROR("CupDetector cannot load param: %s/segment/eps_angle", nh_.getNamespace().c_str() );
    }

    if ( !nh_.getParam("segment/distance_threshold", distance_threshold_) )
    {
        ROS_ERROR("CupDetector cannot load param: %s/segment/distance_threshold", nh_.getNamespace().c_str() );
    }

    if ( !nh_.getParam("debug/publish_table_cloud", publish_table_cloud_) )
    {
        ROS_ERROR("CupDetector cannot load param: %s/debug/publish_table_cloud", nh_.getNamespace().c_str() );
    }

    if ( !nh_.getParam("debug/publish_table_poly", publish_table_poly_) )
    {
        ROS_ERROR("CupDetector cannot load param: %s/debug/publish_table_poly", nh_.getNamespace().c_str() );
    }
}

void
Calibrator::
publishCalibration()
{
    if (calibration_.size() == 2)
    {
        std::vector<float> min;
        min.push_back(calibration_[0].x);
        min.push_back(calibration_[0].y);
        min.push_back(calibration_[0].z);
        nh_.setParam("calibration/table_plane/min", min);

        std::vector<float> max;
        max.push_back(calibration_[1].x);
        max.push_back(calibration_[1].y);
        max.push_back(calibration_[1].z);
        nh_.setParam("calibration/table_plane/max", max);
    }
    else
    {
        std::vector<float> min{0,0,0};
        nh_.setParam("calibration/table_plane/min", min);

        std::vector<float> max{0,0,0};
        nh_.setParam("calibration/table_plane/max", max);
  
    }
}

geometry_msgs::PolygonStamped
Calibrator::
buildTablePoly( pcl::PointXYZRGB minPt, pcl::PointXYZRGB maxPt )
{
    // Initialize message
    geometry_msgs::PolygonStamped table_poly;
    table_poly.header.stamp = ros::Time::now();
    table_poly.header.frame_id = target_frame_id_;
    
    // build rectangle from extreme points 
    geometry_msgs::Point32 vertex;
    vertex.x = minPt.x;
    vertex.y = minPt.y;
    vertex.z = maxPt.z;
    table_poly.polygon.points.push_back(vertex);

    vertex.x = maxPt.x;
    vertex.y = minPt.y;
    vertex.z = maxPt.z;
    table_poly.polygon.points.push_back(vertex);

    vertex.x = maxPt.x;
    vertex.y = maxPt.y;
    vertex.z = maxPt.z;
    table_poly.polygon.points.push_back(vertex);

    vertex.x = minPt.x;
    vertex.y = maxPt.y;
    vertex.z = maxPt.z;
    table_poly.polygon.points.push_back(vertex);

    return table_poly;
}

std::vector<pcl::PointXYZRGB>
Calibrator::
calibrate()
{
    // Store calibration
    std::vector<pcl::PointXYZRGB> min_max;

    if ( !cloud_->empty() )
    {
        // Passthrough filter
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(cloud_);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(passthrough_min_depth_, passthrough_max_depth_);
        pass.filter(*passthrough_cloud_);

        if ( !passthrough_cloud_->empty() )
        {
            // Plane detection
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
            seg.setAxis(Eigen::Vector3f::UnitX());
            seg.setEpsAngle(eps_angle_);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(distance_threshold_);
            seg.setInputCloud(passthrough_cloud_);
            seg.segment (*inliers, *coefficients);             

            // Make sure a plane was found
            if (inliers->indices.size() != 0)
            {
                if ( publish_table_cloud_ )
                {
                    // Set points on plane to red for debugging
                    marked_cloud_ = passthrough_cloud_->makeShared();                

                    for (const auto& idx: inliers->indices)
                    {
                        marked_cloud_->points[idx].r = 0xFF;
                        marked_cloud_->points[idx].g = 0x00;
                        marked_cloud_->points[idx].b = 0x00;
                    }

                    surface_cloud_pub_.publish(marked_cloud_);
                }

                // Extract inliers from cloud
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>());
                pcl::ExtractIndices<pcl::PointXYZRGB> extract;
                extract.setInputCloud(passthrough_cloud_);
                extract.setIndices(inliers);
                extract.setNegative(false);
                extract.filter (*surface_cloud_);
                pcl::PointXYZRGB minPt, maxPt;
                pcl::getMinMax3D (*surface_cloud_, minPt, maxPt);

                // Build table polygon
                geometry_msgs::PolygonStamped table_poly = buildTablePoly( minPt, maxPt );
                table_poly_pub_.publish(table_poly);              
        
                // Set Calibration
                min_max.push_back(minPt); 
                min_max.push_back(maxPt); 
            }
        }
    }

    return min_max;
}


