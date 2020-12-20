# Brobot Cam
ROS package for processing camera related inputs for the brobot pong robot. Includes arduino code 
for reading IMU orientation from the MPU6050 and publishing into ROS.

## MPU6050 Wiring for Arduino Pro-Micro (ATMega32U4)
MPU <---> 32U4
-------------
VCC <---> VCC
GND <---> GND
SCL <--->  3 
SDA <--->  2
INT <--->  7
