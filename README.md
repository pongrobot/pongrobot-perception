# pongrobot_vision
ROS package for processing camera related inputs for the brobot pong robot. Includes arduino code for reading IMU orientation from the MPU6050 and publishing into ROS.

## Dependencies
- rosserial
- tf2
- realsense2_ros
- pcl_ros
### Arduino Dependecies
- rosserial-arduino
- I2CDEV/MPU6050 

## Node Descriptions
- `camera_tf_broadcaster` - read DMP orientation from MPU6050 using rosserial and broadcast the transform from camera frame to robot frame with translation data from config. This is a static tf because the camera should be relatively static during use
- `cup_detector_node` - run the pointcloud based cup detection, takes in the point cloud published to `/camera/depth/color/points` and output a `geometry_msgs/PoseArray` containing the location of all detected cups. A variety of visualization and debugging tools are also provided. The majority of the source code is contained in the `CupDetector` object found in `src/CupDetector.cpp`.

## Published Topics
- `/detector/surface [sensor_msgs/PointCloud2]` - optional topic that publishes a point cloud that shows the detected table surface in red
- `/detector/table [geometry_msgs::PolygonStamped]` - optional topic that publishes a rectangular boundary of the detected table surface
- `/detector/obj [sensor_msgs/PointCloud2]` - optional topic that publishes the segmented objects sitting on the detected table
- `/detector/cluster [sensor_msgs/PointCloud2]` - optional topic that publishes the output of the euclidean clustering algorithm on the objects with a new color for each cluster 
- `/detector/cup_marker [visualization_msgs/MarkerArray]` - optional topic that publishes a cylinder marker at located at the centroid of each detected cup
- `/detector/cup_pose [geometry_msgs/PoseArray]` - topic for the output of the cup detector, each element contains the centroid location of each detected cup ( published in robot frame)

## Config Options
Camera configuration options are available in `config/cam_config.yaml`. In launch files, all config options are loaded as params at `/camera/*`.
- `cam_x` - x offset from robot origin to camera frame origin
- `cam_x` - y offset from robot origin to camera frame origin
- `cam_z` - Offset from robot origin to camera frame origin
- `image_frame_id` - frame_id for image frame
- `camera_frame_id` - frame_id for camera frame

Detector specific config options are available in `config/detector.yaml`
- `passthrough_max_depth` - the upper limit of depth for the passthrough filter on the input cloud in meters
- `passthrough_min_depth`- the lower limit of depth for the passthrough filter on the input cloud in meters
- `obj_max_height` - the height of the tallest object to detect on the table in meters
- `eps_angle` - EPS angle for RANSAC plane detection in radians
- `distance_threshold` - inlier distance threshold for RANSAC plane detection in meters 
- `cluster_tolerance` - the euclidean clustering tolerance in meters
- `min_cluster_size` - the min cluster size for the clustering algorithm
- `max_cluster_size` - the max cluster size for the clustering algorithm

Debugging/visualization config options are also available in `config/detector.yaml`. They are setup to be easily disabled in an effort to preserve processing power in embedded systems.
- `publish_table_cloud` - enable `/detector/surface`
- `publish_table_poly` - enable `/detector/table`
- `publish_obj_cloud` - enable `/detector/obj`
- `publish_cluster_cloud` - enable `/detector/cluster`
- `publish_cup_markers` - enable `/detector/cup_marker`

## Provided Launchfiles
Launchfiles can be found under `/launch` and coordinate running the required nodes and rviz configurations
- `camera_tf.launch` - test node to receive the IMU pose/transform from the Arduino node and visualize it
- `cup_detector.launch` - main node to launch the camera, cup detector and a visualization of the detected cups. If enabled, the additional visualization/debugging data is already configured in rviz although some may be hidden initially.

## Cup Detector Pipeline
Here is a high level overview of the point cloud processing pipeline used to detect the cups in the `cup_detector_node`.
1. __Transform to Robot frame:__ The first step is to pull down the transform from the IMU to make sure the cloud is properly aligned with the ground. This will make it easier to detect the table going forward.
2. __Passthrough Filter:__ Once the cloud is in the correct reference frame, a passthough filter is applied to the depth axis to cut out background noise based on the config. Normally this step would include a downsampling but the realsense doesn't provide a particularly dense cloud in each frame.
3. __RANSAC Plane Detection:__ Next, the cloud is passed into a PCL segmenter using RANSAC to fit against the model of a plane parallel to the depth axis. Knowing that the table is more or less paralllel to the ground in this case is very helpful.
4. __Build the Plane:__ Once the coefficients for the plane model have been determined, the inliers of the cloud are extracted and the extreme values are used to find the approximate bounds of the table.
5. __Filter Out Objects on the Table:__ Knowing the approximate horizontal height of the horizontal dimensions of the table the max height of and inliers on the table, a CubeBox filter can be created to filter out everything from the original cloud except for what is on the table.
6. __Cluster the Objects:__ Once the cloud has been reduced to only the objects on the table, a Euclidean Clustering algorithm is applied to isolate cups or groups of cups.
7. __Calculate Centroid:__ After all the clusters have been detected, the centroid of each cluster can be calculated and used as th location of the cup or set of cups. While the detector cannot always distinguish between cups that are very close together (usually because of the camera FOV and angle) it will still detect the centroid of the group of cups. It happens that this will usually (but not always) be the best location to aim at so this is not a huge problem

## Reference Frames
- Robot Frame: the origin of the robot located near the base
- Camera Frame: reference frame of the camera, transform to robot frame determined from config and IMU data
- Image Frame: image reference with z depth

## MPU6050 Wiring for Arduino Pro-Micro (ATMega32U4)
| MPU     | ATMega32U4  |
| --------| ----------- |
| VCC <---|--> VCC      |
| GND <-- |--> GND      |
| SCL <-- |-->  3       |
| SDA <-- |-->  2       |
| INT <-- |-->  7       |
