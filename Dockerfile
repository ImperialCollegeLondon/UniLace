FROM nvidia/cuda:11.1.1-devel-ubuntu20.04

ARG ROS_PKG=ros_base
ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

# # nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all

SHELL ["/bin/bash", "-c"]

RUN apt-key del 7fa2af80
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/3bf863cc.pub
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/7fa2af80.pub

RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
	python3.8 python3-pip python-is-python3 \
	curl wget zip unzip tar git cmake make build-essential \
	gnupg2 \
	lsb-release \
	ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# ####################################################################################################
# ######################################### ROS INSTALLATION #########################################
# ####################################################################################################

# Add the ROS repository and the ROS key
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-${ROS_DISTRO}.list' \
     && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-core \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN apt-get update && apt-get install -y python3-rosdep \
    && rosdep init \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# Setup the environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Install additional dependencies for building ROS packages
RUN apt-get update && apt-get install -y \
	ros-${ROS_DISTRO}-catkin \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
	python3-tk \
	python3-catkin-tools \
    build-essential \
    && rm -rf /var/lib/apt/lists/*


# ####################################################################################################
# ######################################### PYTHON RELATED ###########################################
# ####################################################################################################
RUN python3 -m pip install --user --upgrade pip>=20.3 numpy>=1.18.0

RUN	python3 -m pip install --user \
	open3d \
	opencv-contrib-python-headless \
	pyyaml \
	scipy \
	scikit-image \
	matplotlib \
	gymnasium

# ####################################################################################################
# ###################################### BASELINE INSTALLATION #######################################
# ####################################################################################################
RUN apt-get update && \
	DEBIAN_FRONTEND=noninteractive apt-get install -y \
	ros-${ROS_DISTRO}-control-toolbox \
	ros-${ROS_DISTRO}-controller-interface \
	ros-${ROS_DISTRO}-controller-manager \
	ros-${ROS_DISTRO}-effort-controllers \
	ros-${ROS_DISTRO}-force-torque-sensor-controller \
	ros-${ROS_DISTRO}-gazebo-ros-control \
	ros-${ROS_DISTRO}-joint-limits-interface \
	ros-${ROS_DISTRO}-joint-state-publisher \
	ros-${ROS_DISTRO}-joint-state-controller \
	ros-${ROS_DISTRO}-joint-trajectory-controller \
	ros-${ROS_DISTRO}-moveit \
	ros-${ROS_DISTRO}-moveit-commander \
	ros-${ROS_DISTRO}-moveit-core \
	ros-${ROS_DISTRO}-moveit-planners \
	ros-${ROS_DISTRO}-moveit-ros-move-group \
	ros-${ROS_DISTRO}-moveit-ros-planning \
	ros-${ROS_DISTRO}-moveit-ros-visualization \
	ros-${ROS_DISTRO}-moveit-simple-controller-manager \
	ros-${ROS_DISTRO}-position-controllers \
	ros-${ROS_DISTRO}-rqt-joint-trajectory-controller \
	ros-${ROS_DISTRO}-transmission-interface \
	ros-${ROS_DISTRO}-velocity-controllers \
	ros-${ROS_DISTRO}-simple-message \
	ros-${ROS_DISTRO}-hector-xacro-tools \
	ros-${ROS_DISTRO}-robot-state-publisher \ 
	ros-${ROS_DISTRO}-ros-numpy && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /software

# install fast-downward
RUN git clone -b release-20.06 https://github.com/aibasel/downward.git 
RUN cd /software/downward && \
	./build.py

# ####################################################################################################
# ########################################### FINALISATION ###########################################
# ####################################################################################################

ENTRYPOINT [ "" ]

WORKDIR /catkin_ws
CMD /bin/bash

