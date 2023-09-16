#!/bin/bash

echo "=========================="
echo "Stopping App 3DM-G"

systemctl stop rosnodeChecker
systemctl stop microstrain-3dmg-ros

