# pongrobot_vision
ROS package for processing camera related inputs for the brobot pong robot. Includes arduino code for reading IMU orientation from the MPU6050 and publishing into ROS.

## Node Descriptions
- orientation_to_pose - read orientation from MPU6050 using DPM and publish over rosserial
- realsense - low budget ros wrapper for D415 realsense camera with color alignment. It has less functionality than the realsense_camera package but it's more simple to use and isn't currently failing most of it's integration tests for Noetic

## Dependencies
- OpenCV4 and ros cv_bridge
- librealsense2
- rosserial
- tf2
- image_transport

## Published Topics
- `/camera/info [sensor_msgs/CameraInfo]` - camera intrinsics and metadata for all camera data
- `/camera/color [sensor_msgs/Image]` - brg8 image 
- `/camera/depth [sensor_msgs/Image]` - 32FC1 depth data aligned to color stream in meters
- `/camera/depth_raw [sensor_msgs/Image]` - 16Uc1 depth data aligned to color in mm (openni)
- `/camera/depth_map [sensor_msgs/Image]` - [OPTIONAL] bgr8 colorized representation of depth data for visualization 
- `/camera/imu_orientation [geomtry_msgs/Orientation` - camera orientation from external IMU for transform using DPM

## Config Options
Camera configuration options are available in `config/cam_config.yaml`. In launch files, all config options are loaded as params at `/camera/*`.
- `cam_x` - x offset from robot origin to camera frame origin
- `cam_x` - y offset from robot origin to camera frame origin
- `cam_z` - Offset from robot origin to camera frame origin
- `image_frame_id` - frame_id for image frame
- `camera_frame_id` - frame_id for camera frame
- `enable_depth_map` - enable or disable colorized depth map for visialization

## MPU6050 Wiring for Arduino Pro-Micro (ATMega32U4)
| MPU     | ATMega32U4  |
| --------| ----------- |
| VCC <---|--> VCC      |
| GND <-- |--> GND      |
| SCL <-- |-->  3       |
| SDA <-- |-->  2       |
| INT <-- |-->  7       |
