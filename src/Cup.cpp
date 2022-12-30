#include "Cup.h"

Cup::
Cup(const uint16_t cup_id, const std::string frame_id):
    id_(cup_id)
{
    lastDetection_ = ros::Time::now();

    estimated_pose_.header.seq = 0;
    estimated_pose_.header.stamp = ros::Time::now();
    estimated_pose_.header.frame_id = frame_id;

    estimated_pose_.pose.orientation.w = 1;
    estimated_pose_.pose.orientation.x = 0;
    estimated_pose_.pose.orientation.y = 1;
    estimated_pose_.pose.orientation.z = 0;
}

uint16_t
Cup::
getId()
{
    return id_;
}

bool
Cup::
insertDetection( const geometry_msgs::PoseStamped detection )
{
    // Make sure the detection is in the expected frame
    if (detection.header.frame_id != estimated_pose_.header.frame_id )
    {
        ROS_WARN("Cup %d: Attempting to assign detection with invalid frame id (%s)", id_, detection.header.frame_id.c_str());
        return false;
    }

    // Ingest the detection
    detections_.push_front(detection);
    lastDetection_ = detection.header.stamp;

    // Expire old data to only keep the most recent detections
    while ( detections_.size() > MAX_DETECTIONS_PER_CUP )
    {
        detections_.pop_back();
    }

    // Calculate new mean
    double x, y, z = 0;
    std::list<geometry_msgs::PoseStamped>::iterator it;
    for(it = detections_.begin(); it != detections_.end(); ++it)
    {
        x += it->pose.position.x;
        y += it->pose.position.y;
        z += it->pose.position.z;
    }

    x = x/detections_.size();
    y = y/detections_.size();
    z = z/detections_.size();

    // Update internal pose
    estimated_pose_.header.stamp = ros::Time::now();
    estimated_pose_.pose.position.x = x;
    estimated_pose_.pose.position.y = y;
    estimated_pose_.pose.position.z = z;

    // TODO: Calculate variance/other useful stats

    return true;
}

ros::Time
Cup::
lastDetectionTime()
{
    return lastDetection_;
}

geometry_msgs::PoseStamped
Cup::
getPose()
{
    return estimated_pose_;
}
