FROM ubuntu:24.04
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
LABEL maintainer="UTN FRBA Robotica"

ENV ROS2_DISTRO="jazzy"
ENV USER="utn_robotica"
RUN useradd -ms /bin/bash $USER
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

USER $USER
RUN export LANG=en_US.UTF-8

## Enable required repositories
USER root

RUN apt install -y software-properties-common
RUN add-apt-repository universe
RUN apt update
RUN apt install -y curl
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

## Install development tools
RUN apt update
RUN apt install -y nano emacs
RUN apt install -y wget gpg
RUN wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
RUN install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
RUN echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | tee /etc/apt/sources.list.d/vscode.list > /dev/nulla
# RUN https://packages.microsoft.com/repos/code stable main" |sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null
RUN apt install -y apt-transport-https
RUN apt update
RUN apt install -y code
RUN apt install -y ros-dev-tools

## Install ROS2
RUN apt update
RUN apt upgrade
RUN apt install -y ros-jazzy-desktop

## Setup enviroment
USER $USER
RUN source /opt/ros/jazzy/setup.sh

# INSTALLING iRobotCreate
USER root
## Dependencies
RUN apt install -y python3-colcon-common-extensions
RUN apt install -y python3-rosdep
RUN apt install -y python3-vcstool
## Gazebo Harmonic
RUN apt-get update
RUN apt-get install -y wget
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt-get update
RUN apt install -y ros-jazzy-ros-gz

## Building
### Create workspace
USER $USER
RUN mkdir -p /home/utn_robotica/create3_ws/src
RUN cd /home/utn_robotica/create3_ws/src
RUN git clone https://github.com/iRobotEducation/create3_sim.git /home/utn_robotica/create3_ws/src/create3_sim/
RUN git clone https://github.com/iRobotEducation/irobot_create_msgs.git /home/utn_robotica/create3_ws/src/irobot_create_msgs/
RUN cd ~/create3_ws

### Open Navigation Packages 
USER root
RUN git clone https://github.com/open-navigation/opennav_coverage.git /opt/ros/jazzy/opennav_coverage/
RUN git clone https://github.com/open-navigation/slam_toolbox.git /opt/ros/jazzy/slam_toolbox/
RUN git clone https://github.com/open-navigation/robot_localization.git /opt/ros/jazzy/robot_localization/
RUN git clone https://github.com/open-navigation/gz2-toy-demos.git /opt/ros/jazzy/gz2-toy-demos/
RUN git clone https://github.com/open-navigation/opennav_amd_demonstrations.git /opt/ros/jazzy/opennav_amd_demonstrations/
RUN git clone https://github.com/open-navigation/opennav_docking.git /opt/ros/jazzy/opennav_docking/
RUN git clone https://github.com/open-navigation/launch_ros.git /opt/ros/jazzy/launch_ros/

USER root
RUN apt-get update
RUN source /opt/ros/jazzy/setup.sh
RUN rosdep init

USER $USER
RUN cd ~/create3_ws
RUN source /opt/ros/jazzy/setup.sh
# RUN rosdep update
# RUN rosdep install --from-path ~/create3_ws/src -yi
# RUN colcon build --symlink-install
# RUN source install/local_setup.bash

USER root
RUN apt install libxcb-cursor0
RUN apt install xcb
RUN apt-get update && \
    apt-get install -y libqt5gui5 && \
    rm -rf /var/lib/apt/lists/*
ENV QT_DEBUG_PLUGINS=1

# USER $USER

CMD ["/bin/bash"]
