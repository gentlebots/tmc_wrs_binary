# Copyright (c) 2019 TOYOTA MOTOR CORPORATION
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

#  * Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#  notice, this list of conditions and the following disclaimer in the
#  documentation and/or other materials provided with the distribution.
#  * Neither the name of Toyota Motor Corporation nor the names of its
#  contributors may be used to endorse or promote products derived from
#  this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
FROM ros:melodic

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update; apt-get install curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update --fix-missing && \
    apt-get install -y curl apt-transport-https python-pip && \
    apt-get clean

# OSRF distribution is better for gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    curl -L http://packages.osrfoundation.org/gazebo.key | apt-key add -

# install depending packages
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install --no-install-recommends -y \
    ros-melodic-gazebo-ros ros-melodic-gazebo-plugins ros-melodic-gazebo-ros-control libgazebo9-dev libignition-transport4-dev libpoco-dev python-scipy libgsl-dev \
    ros-melodic-dwa-local-planner \
    ros-melodic-base-local-planner \
    ros-melodic-eigen-conversions \
    ros-melodic-robot-state-publisher \
    ros-melodic-moveit-core \
    ros-melodic-moveit-* \
    ros-melodic-urdfdom-py \
    ros-melodic-roslint \
    ros-melodic-joint-state-controller \
    ros-melodic-joint-trajectory-controller \
    ros-melodic-move-base \
    ros-melodic-map-server \
    ros-melodic-vision-msgs \
    ros-melodic-xacro \
    ros-melodic-control-toolbox \
    ros-melodic-four-wheel-steering-msgs \
    ros-melodic-perception-pcl \
    ros-melodic-realtime-tools \
    ros-melodic-urdf-geometry-parser \
    ros-melodic-joint-state-publisher \
    liburdfdom-tools \
    ros-melodic-image-proc \
    ros-melodic-depth-image-proc \
    ros-melodic-effort-controllers \
    ros-melodic-ros-controllers \
    ros-melodic-pcl-ros \
    ros-melodic-tf-conversions \
    ros-melodic-four-wheel-steering-msgs \
    ros-melodic-realtime-tools \
    ros-melodic-navigation \
    ros-melodic-navigation* \
    ros-melodic-moveit-ros-perception && \
    pip install -U --ignore-installed pyassimp supervisor supervisor_twiddler && \
    apt-get autoremove -y && \
    apt-get clean

RUN mkdir /wrs_ws
ADD src /wrs_ws/src
ADD gentlebots_startup.sh /wrs_ws/src/gentlebots_startup.sh

RUN cd /wrs_ws/src && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_init_workspace || true
RUN cd /wrs_ws && source /opt/ros/$ROS_DISTRO/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN cd /wrs_ws && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make install -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO -DCATKIN_ENABLE_TESTING=0

# Install DOPE & Moveit deps
RUN python -m pip install -U \
  pyrr==0.9.2 \
  torch==0.4.0 \
  numpy==1.14.2 \
  scipy==1.1.0 \
  opencv_python==3.4.1.15 \
  Pillow==5.3.0 \
  torchvision==0.2.1 \
  pyassimp==4.1.3
# Install ROS2 eloquent from source

## Set locale

RUN locale && \
apt update && apt install locales -y && \
locale-gen en_US en_US.UTF-8 && \
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
export LANG=en_US.UTF-8 && \
locale

## Add the ROS2 apt repository

RUN apt update && apt install curl gnupg2 lsb-release -y && \
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

## Install development tools and ROS tools

RUN apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python-rosdep \
  python3-vcstool \
  wget \
  screen
  
# install some pip packages needed for testing
RUN python3 -m pip install -U \
  argcomplete \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  pytest-cov \
  pytest-runner \
  setuptools
# install Fast-RTPS dependencies
RUN apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
RUN apt install --no-install-recommends -y \
  libcunit1-dev

RUN rosdep update

## Create Workspace and import the packages and dependencies

RUN mkdir -p /eloquent_ws/src && \
cd /eloquent_ws && \
wget https://raw.githubusercontent.com/ros2/ros2/eloquent/ros2.repos && \
vcs import src < ros2.repos

## Replace the ros1_bridge by own dedicated bridges

RUN rm -rf /eloquent_ws/src/ros2/ros1_bridge

RUN cd /eloquent_ws/src/ros2 && \
git clone -b dedicated_bridges_eloquent --recursive https://github.com/IntelligentRoboticsLabs/ros1_bridge.git

USER root

RUN cd /eloquent_ws && \
rosdep install --from-paths src --ignore-src --rosdistro eloquent -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"

### Install tf2_msgs

RUN apt-get install ros-melodic-tf2-msgs 


# Compile the ROS2 Workspace

RUN cd /eloquent_ws && \
colcon build --symlink-install --packages-skip ros1_bridge

RUN mkdir -p /eloquent_moveit_ws/src && \
cd /eloquent_moveit_ws && \
wget https://raw.githubusercontent.com/gentlebots/tmc_wrs_binary/ros2/moveit_msgs.repos && \
vcs import src < moveit_msgs.repos

RUN source /eloquent_ws/install/setup.bash && \
cd /eloquent_moveit_ws && \
colcon build --symlink-install

# Source ROS1/ROS2 Environment and compile ros1_bridge

##ENV MAKEFLAGS -j1

RUN source /opt/ros/melodic/setup.bash && \
source /eloquent_moveit_ws/install/setup.bash && \
cd /eloquent_ws && \
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure

ADD entrypoint-wrs.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

#ADD filterable-rosmaster.py /opt/ros/melodic/bin/
#RUN rm /opt/ros/$ROS_DISTRO/bin/rosmaster && ln -s /opt/ros/$ROS_DISTRO/bin/filterable-rosmaster.py /opt/ros/$ROS_DISTRO/bin/rosmaster

RUN source /opt/ros/$ROS_DISTRO/setup.bash && rosrun tmc_gazebo_task_evaluators setup_score_widget

ADD supervisord.conf /etc/supervisor/supervisord.conf

VOLUME [ \
    "/opt/ros/melodic/share/hsrb_description", \
    "/opt/ros/melodic/share/hsrb_meshes", \
    "/opt/ros/melodic/share/tmc_wrs_gazebo_worlds", \
    "/opt/ros/melodic/share/gazebo_ros", \
    "/opt/ros/melodic/lib/gazebo_ros", \
    "/opt/ros/melodic/lib/python2.7/dist-packages/gazebo_ros", \
    "/opt/ros/melodic/lib/python2.7/dist-packages/gazebo_msgs", \
    "/opt/ros/melodic/share/hsrb_rosnav_config", \
    "/opt/ros/melodic/share/tmc_control_msgs", \
    "/opt/ros/melodic/lib/python2.7/dist-packages/tmc_control_msgs", \
    "/opt/ros/melodic/include/tmc_control_msgs" \
    ]

CMD ["/usr/local/bin/supervisord", "-n", "-c", "/etc/supervisor/supervisord.conf"]

