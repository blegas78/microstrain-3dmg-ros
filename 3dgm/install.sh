#!/bin/bash

echo "=========================="
echo "Installing App 3dmg"

# Here is where we perform installation of scripts, services, etc.
echo " - Installing ROS packages for 3DM-G..."

LIBPANDA_SRC=$(cat /etc/libpanda.d/libpanda_src_dir)
LIBPANDA_USER=$(cat /etc/libpanda.d/libpanda_usr)
LAUNCH_FILE=microstrain_record.launch

source /home/$LIBPANDA_USER/.bashrc

if [ -d /home/$LIBPANDA_USER/strym ]; then
    pushd /home/$LIBPANDA_USER/strym
    git pull
else
    pushd /home/$LIBPANDA_USER/
    git clone https://github.com/jmscslgroup/strym
fi
popd

runuser -l $LIBPANDA_USER -c /etc/libpanda.d/apps/microstrain-3dmg-ros/install3dmg.sh

