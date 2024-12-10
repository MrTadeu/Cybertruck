#!/bin/bash

# --------------------------- ROS2 uninstallation --------------------------- (https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
# 
sudo apt remove ~nros-jazzy-* && sudo apt autoremove

sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt autoremove
# Consider upgrading for packages previously shadowed.
sudo apt upgrade

# ----------------------------------------------------------------------------
