#!/bin/bash

set -e

source /root/px4_ros_com_ros2/install/setup.bash 
sleep 20
micrortps_agent start -t ${RTPS_PROTOCOL} -i ${PX4_IP} > /dev/null &
ros2 run px4_ros_com offboard_control

# colcon build --symlink-install --packages-skip px4_msgs px4_ros_com
#
exec $@

