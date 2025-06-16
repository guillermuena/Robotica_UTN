FROM ubuntu:24.04
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
LABEL maintainer="UTN FRBA Robotica"

ENV ROS2_DISTRO="jazzy"
ENV USER="utn_robotica"
RUN useradd -m -s /bin/bash $USER
USER $USER
WORKDIR /home/utn_robotica
ENV DISPLAY=localhost:0


# SETUP ENVIROMENT
## Set Locale
USER root
RUN apt update
RUN apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN apt-get update && apt-get install -y sudo
RUN echo 'utn_robotica ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER $USER
RUN export LANG=en_US.UTF-8

## Enable required repositories
RUN sudo apt install -y software-properties-common
RUN sudo add-apt-repository universe
RUN sudo apt update
RUN sudo apt install -y curl
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN sudo echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

## Install development tools
RUN sudo apt update
RUN sudo apt install -y nano emacs
RUN sudo apt install -y wget gpg
RUN sudo wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
RUN sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
RUN sudo echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null
# RUN https://packages.microsoft.com/repos/code stable main" |sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null
RUN sudo apt install -y apt-transport-https
RUN sudo apt update
RUN sudo apt install -y code
RUN sudo apt install -y ros-dev-tools

## Install ROS2
RUN sudo apt update
RUN sudo apt upgrade
RUN sudo apt install -y ros-jazzy-desktop

## Setup enviroment
USER $USER
RUN source /opt/ros/jazzy/setup.sh

## Dependencies
RUN sudo apt install -y python3-colcon-common-extensions
RUN sudo apt install -y python3-rosdep
RUN sudo apt install -y python3-vcstool
## Gazebo Harmonic
RUN sudo apt-get update
RUN sudo apt-get install -y wget
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
USER root
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
USER $USER
RUN sudo apt-get update
RUN sudo apt install -y ros-jazzy-ros-gz
RUN sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers

## Building
### Create workspace
RUN mkdir -p /home/utn_robotica/ros_ws/src

### Create3 simulation code
RUN git clone https://github.com/iRobotEducation/create3_sim.git /home/utn_robotica/ros_ws/src/create3_sim/
RUN git clone https://github.com/iRobotEducation/irobot_create_msgs.git /home/utn_robotica/ros_ws/src/irobot_create_msgs/

### Open Navigation Packages 
RUN git clone https://github.com/open-navigation/opennav_coverage.git /home/utn_robotica/ros_ws/src/open-navigation/opennav_coverage
RUN git clone https://github.com/open-navigation/slam_toolbox.git /home/utn_robotica/ros_ws/src/open-navigation/slam_toolbox
RUN git clone https://github.com/open-navigation/robot_localization.git /home/utn_robotica/ros_ws/src/open-navigation/robot_localization
RUN git clone https://github.com/open-navigation/gz2-toy-demos.git /home/utn_robotica/ros_ws/src/open-navigation/gz2-toy-demos
# RUN git clone https://github.com/open-navigation/opennav_amd_demonstrations.git /home/utn_robotica/ros_ws/src/open-navigation/opennav_amd_demonstrations
RUN git clone https://github.com/open-navigation/opennav_docking.git /home/utn_robotica/ros_ws/src/open-navigation/opennav_docking
RUN git clone https://github.com/open-navigation/launch_ros.git /home/utn_robotica/ros_ws/src/open-navigation/launch_ros

### ROS2 control
RUN cd /home/utn_robotica/ros_ws/src
RUN git clone https://github.com/ros-controls/ros2_control_demos
RUN cd /home/utn_robotica/ros_ws/
RUN sudo apt-get update
RUN source /opt/ros/jazzy/setup.sh
RUN sudo rosdep init
RUN rosdep update --rosdistro=jazzy
RUN rosdep install --from-paths ./ -i -y --rosdistro jazzy


USER root
RUN apt install libxcb-cursor0
RUN apt install xcb
RUN apt-get update && \
    apt-get install -y libqt5gui5 && \
    rm -rf /var/lib/apt/lists/*
ENV QT_DEBUG_PLUGINS=1

USER $USER
CMD ["/bin/bash"]
