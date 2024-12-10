#!/bin/bash

ROS_DISTRO=jazzy

# --------------------------- ROS2 installation --------------------------- (https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
# 
locale  

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install ros-dev-tools

sudo apt update

sudo apt upgrade

sudo apt install ros-$ROS_DISTRO-desktop-full # RVIZ tbm é instalado e pode ocorrer um erro no terminal do vscode (instalar o .deb) https://github.com/ros2/ros2/issues/1406

sudo apt install ros-$ROS_DISTRO-ros-base

# adding ROS2 environment variables
CMD="source /opt/ros/jazzy/setup.bash"

# if exists in ~/.bashrc
if ! grep -Fxq "$CMD" ~/.bashrc; then
    echo "Adicionando o comando ao ~/.bashrc..."
    echo "$CMD" >> ~/.bashrc
else
    echo "O comando já está presente no ~/.bashrc."
fi

# load ~/.bashrc
source ~/.bashrc

# ----------------------------------------------------------------------------

# --------------------------- RQT installation --------------------------- (https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html#install-rqt)
sudo apt update

sudo apt install 'ros-jazzy-rqt*'

# ----------------------------------------------------------------------------
# --------------------------- Joint GUI installation --------------------------- 

sudo apt install ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui

# ----------------------------------------------------------------------------
