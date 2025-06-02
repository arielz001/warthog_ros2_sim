#!/bin/zsh

set -e  # stop the script if any command fails

echo "Updating package list..."
sudo apt update

echo "Installing ros-humble-ros2-controllers and ros-humble-ros2-control..."
sudo apt install -y ros-humble-ros2-controllers ros-humble-ros2-control

echo "Installing ros-humble-ign-ros2-control..."
sudo apt-get install -y ros-humble-ign-ros2-control

echo "Installation completed!"
