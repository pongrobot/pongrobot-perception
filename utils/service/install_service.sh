# Install system service to run the launch file at startup
# See http://docs.ros.org/en/jade/api/robot_upstart/html/install.html

echo "Installing pong robot system service..."
rosrun robot_upstart install \
	--job pongrobot \
	--setup /home/bro/catkin_ws/devel/setup.sh \
	--logdir /var/log/upstart \
	--symlink \
	pongrobot_perception/launch/pongrobot_headless.launch
