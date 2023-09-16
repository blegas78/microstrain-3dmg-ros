#!/bin/bash
# Author: Matt Bunting, Matt Nice

echo "----------------------------"
if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

source ~/.bashrc
LIBPANDA_SRC=$(cat /etc/libpanda.d/libpanda_src_dir)
LIBPANDA_USER=$(cat /etc/libpanda.d/libpanda_usr)
LAUNCH_FILE=microstrain_record.launch

cd ~
if [ ! -d catkin_ws/src ]; then
    mkdir -p catkin_ws/src
#    source /opt/ros/noetic/setup.bash   # will it always be noetic, in this location?
fi
#cd catkin_ws/src


#cd ~/catkin_ws
source /opt/ros/noetic/setup.bash

# can_to_ros should be provided by libpanda-apps.yaml
echo "Regenerating CanToRos"
cd ~/catkin_ws/src/can_to_ros/scripts
echo y | ./regenerateCanToRos.sh


# Build:
cd /home/${LIBPANDA_USER}/catkin_ws
catkin_make
source /home/${LIBPANDA_USER}/catkin_ws/devel/setup.sh

echo "Installing 3DM-G..."

pushd /home/${LIBPANDA_USER}/catkin_ws
source devel/setup.sh

rosrun robot_upstart install microstrain-3dmg-ros/launch/${LAUNCH_FILE} --user root

echo "Enabling microstrain_records startup script"
sudo systemctl daemon-reload
sudo systemctl enable can
popd




echo "----------------------------"
