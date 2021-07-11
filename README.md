# pongrobot_perception 1.0.0
ROS package for processing camera related inputs for the brobot pong robot. Includes arduino code for reading IMU orientation from the MPU6050 and publishing into ROS. This tag should be run against pongrobot_actuation 1.0.0 for best compatibility.

## Dependencies
- rosserial
- tf2
- realsense2_ros_camera
- realsense2_ros_description
- librealsense2
- pcl_ros
### Arduino Dependecies
- rosserial-arduino
- I2CDEV/MPU6050
- VL53L0X (Pololu)

### A Note on Realsense
There are currently problems with running the current release of realsense on ARM64 distributed through apt when the camera is configured to use both rgb and stereo depth simultaneously. To resolve this, we recommend building and installing librealsense2 and realsense-ros from source and using the ROS wrapper tag [realsense-ros 2.2.24](https://github.com/IntelRealSense/realsense-ros/tree/2.2.24) and the corresponding SDK tag [librealsense v2.44.0](https://github.com/IntelRealSense/librealsense/tree/v2.44.0)

## Node Descriptions
- `tf_broadcaster` - read DMP orientation from MPU6050 and height from VL53L0X using rosserial and broadcast all the transforms on the robot.
- `cup_detector_node` - run the pointcloud based cup detection, takes in the point cloud published to `/camera/depth/color/points` and output a `geometry_msgs/PoseArray` containing the location of all detected cups. A variety of visualization and debugging tools are also provided. The majority of the source code is contained in the `CupDetector` object found in `src/CupDetector.cpp`.
- `game_logic_node` - handle all the game logic for determining the state of the table, picking a target and requesting shot from the launcher. 

## Topics
- `/detector/surface [sensor_msgs/PointCloud2]`- optional topic that publishes a point cloud that shows the detected table surface in red
- `/detector/table [geometry_msgs::PolygonStamped]` - optional topic that publishes a rectangular boundary of the detected table surface
- `/detector/obj [sensor_msgs/PointCloud2]` - optional topic that publishes the segmented objects sitting on the detected table
- `/detector/cluster [sensor_msgs/PointCloud2]` - optional topic that publishes the output of the euclidean clustering algorithm on the objects with a new color for each cluster 
- `/detector/cup_marker [visualization_msgs/MarkerArray]` - optional topic that publishes a cylinder marker at located at the centroid of each detected cup
- `/detector/cup_array [geometry_msgs/PoseArray]` - topic for the output of the cup detector, each element contains the centroid location of each detected cup ( published in robot frame)
- `/detector/calibrate [std_msgs/EmptyMsg]` - signal to activate the calibration routine, this will send updated detector parameters to the parameter server (see pongrobot_perception#4)
- `/detector/restart [std_msgs/EmptyMsg]` - signal to restart the cup detector. This will trigger a parameter reload, applying the new calibration to the detector
- `/restart [std_msgs/EmptyMsg]` - signal to restart the Game manager, this will reset an persistent game state data
- `/calibration/request [std_msgs/EmptyMsg]` - signal to request calibration from the calibration node
- `/calibration/complete [std_msgs/EmptyMsg]` - signal from the calibration node to notify the game manager that the calibration sequence has been completed.
- `/launcher/target_pose [geometry_msgs/PoseStamped]` - the topic for the target cup to hit, (published in launcher frame)
- `/launcher/has_ball [std_msgs/Bool]` - signal from the launcher telling if a ball is present. Used by the Game Manager to trigger a shot
- `/launcher/shot [geometry_msgs/PoseStamped]` - signal sent from the launcher to confirm that a shot has completed. The location of the shot is included to discriminate between commands

## Config Options
Node rate config options are available in `config/rate_config.yaml` and loaded under the namespace `/rate`
detector: 10.0 # rate to run the cup detector note (hz)
game: 10.0 # rate to run the game node at (hz)
tf_broadcast: 20.0 # rate to run the game node at (hz)

Coordinate frame config options are available in `config/frame_config.yaml` and loaded under the namespace `/frame`
- `camera_frame_id`: camera frame id
- `world_frame_id`: world frame id
- `robot_base_frame_id`: robot base id
- `robot_center_frame_id`: robot center id
- `launcher_frame_id`: launcher
- `geometry/camera_x_offset`: X offset from robot center to camera frame
- `geometry/camera_z_offset`: Z offset from robot center to camera frame
- `geometry/launcher_z_offset`: Z offset from robot center to launcher frame 

Cup Detector config options are available in `config/detector_config.yaml` and loaded under the namespace `/detector`
- `target_frame_id`: world # frame to perform vision processing in
- `filter/passthrough_max_depth`: background clipping dist in meters
- `filter/passthrough_min_depth`: foreground clipping dist in meters
- `filter/object_max_height`: how far above the detected table to put the top of the box, should be greater than the cup height
- `segment/eps_angle`: EPS angle for RANSAC plane detection in radians
- `segment/distance_threshold`: how close a point must be to be an inlier in meters (REP 103)
- `cluster/tolerance`: the euclidean clustering tolerance in meters
- `cluster/min_cluster_size`: smallest allowable point cluster
- `cluster/max_cluster_size`: largest allowable point cluster
- `debug/publish_table_cloud`: Show the detected table in red
- `debug/publish_table_poly`: Show the bounds of the detected table
- `debug/publish_obj_cloud`: Show the objects on the table
- `debug/publish_cluster_cloud`: Show the output of the clustering segmentation
- `debug/publish_cup_markers`: Show the detected cups as cylinder markers

Game Manager config options are available in `config/game_config.yaml` and loaded under the namespace `/game`
`calibration_timeout`: The max amount of time the game node will wait for calibration to complete (sec)
`launcher_timeout`: The max amount of time the game node will wait for launcher confirmation before assuming failure (sec)
`cup_height`: The height of the cup, used to calculate the z component of the target (will be removed in a future version)


## Provided Launchfiles
Launchfiles can be found under `/launch` and coordinate running the required nodes and rviz configurations. Headless variations of many launchfiles are provided, the only difference is these do not run rviz.
- `pongrobot_tf.launch` - generate all the robot transforms anf visualize them in rviz
- `cup_detector.launch` - main node to launch the camera, cup detector and a visualization of the detected cups. If enabled, the additional visualization/debugging data is already configured in rviz although some may be hidden initially.
- `perception.launch` - launch all the nodes under the perception package including the GameManager, TF broadcaster and the CupDetector
  
## Utilities
Some useful utilities are provided under `/utils`, these are used for setting up system services, remote connections and udev rules.

### Udev Rules
Before the package can be run, the appropriate udev rules must be setup. This will allow the system to correctly identify USB devices. To setup the rule for the transform node, run `sudo cp 11-brobot-tf.rules /etc/udev/rules.d`. *NOTE:* Udev rules for the realsense are not included as they should be installed with the library.

### Setup Remote Environment
The system is setup so the robot will normally run without rendering the visualization but it will stream all the necessary data to do so on a remote machine. If ROS is properly configured on another computer on the same network, all the visualization and debugging tools such as rviz can connect to the robot. In order to speed up this process, a shell script has been provided to quickly set the necessary environment variables for remove visualization. To enable remove monitoring, run `source utils/setupRemoteEnv.sh` from the package root.

### Running as a Background Service
The launch files can be installed as a system service, to run in the background on startup. This is managed using the [robot_upstart](http://wiki.ros.org/robot_upstart) package. 

The `/utils/service/` folder contains several scripts to manage this process:
- `install_service.sh` uses `robot_upstart` to install a system service that will run `pongrobot_headless.launch`.
- `start_service.sh` starts the system service. This setting persists across restarts.
- `stop_service.sh` stops the system service. This setting persists across restarts. Run this command before you manually run any launch files, to prevent the background service conflicting.
- `uninstall_service.sh` uninstalls the system service. Only needed if the configuration in `install_service.sh` is changed.

## Cup Detector Pipeline
Here is a high level overview of the point cloud processing pipeline used to detect the cups in the `cup_detector_node`.
1. __Transform to Robot frame:__ The first step is to pull down the transform from the IMU to make sure the cloud is properly aligned with the ground. This will make it easier to detect the table going forward.
2. __Passthrough Filter:__ Once the cloud is in the correct reference frame, a passthough filter is applied to the depth axis to cut out background noise based on the config. Normally this step would include a downsampling but the realsense doesn't provide a particularly dense cloud in each frame.
3. __RANSAC Plane Detection:__ Next, the cloud is passed into a PCL segmenter using RANSAC to fit against the model of a plane parallel to the depth axis. Knowing that the table is more or less parallel to the ground in this case is very helpful.
4. __Build the Plane:__ Once the coefficients for the plane model have been determined, the inliers of the cloud are extracted and the extreme values are used to find the approximate bounds of the table.
5. __Filter Out Objects on the Table:__ Knowing the approximate horizontal height of the horizontal dimensions of the table the max height of and inliers on the table, a CubeBox filter can be created to filter out everything from the original cloud except for what is on the table.
6. __Cluster the Objects:__ Once the cloud has been reduced to only the objects on the table, a Euclidean Clustering algorithm is applied to isolate cups or groups of cups.
7. __Calculate Centroid:__ After all the clusters have been detected, the centroid of each cluster can be calculated and used as th location of the cup or set of cups. While the detector cannot always distinguish between cups that are very close together (usually because of the camera FOV and angle) it will still detect the centroid of the group of cups. It happens that this will usually (but not always) be the best location to aim at so this is not a huge problem

## Reference Frames
- World Frame: the origin is on the ground at the base of the center of the robot, Z is normal to the ground
- Robot Base Frame: The same origin as World Frame with pitch and roll attitude of the robot calculated from the IMU. __NOTE: yaw component is removed from the transform__
- Robot Center Frame: the same attitude as Robot Base frame but the origin is translated up in Z to align with the base of the robots aluminum frame
- Launcher Frame: reference frame centered at the center of the launcher. Z offset from Robot Center is defined in the config
- Camera Frame: reference frame of the camera, transform to Robot Center frame determined from config and IMU data
- Image Frame: image reference with z depth
  
| MPU      | ATMega32U4  | VL53L0X  |
| -------- | ----------- | -------- |
| VCC <--- | --> VCC <-- | ---> VCC |
| GND <--  | --> GND <-- | ---> GND |
| SCL <--  | -->  3  <-- | ---> SCL |
| SDA <--  | -->  2  <-- | ---> SDA |
| INT <--  | -->  7      | NC       |
