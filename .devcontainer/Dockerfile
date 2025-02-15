# Base image
FROM nvidia/cudagl:11.3.1-devel-ubuntu20.04
# Environment variables
ENV ROS_DISTRO=noetic
ENV ROS_PKG=desktop-full
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Amsterdam

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

# Install the first essential packages 
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    software-properties-common \
    git \
    cmake \
    curl \
    wget \
    lsb-release \
    ca-certificates \
    build-essential \
    unzip \
    net-tools \
    nano \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

# Add ROS to sources list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS, rosdep, pip and Catkin tools
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-${ROS_PKG} \
    python3-catkin-tools \
    python3-rosdep \
    python3-pip \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init \
    && rosdep update

# Make GitHub connection to clone private repositories
RUN mkdir -p -m 0600 ~/.ssh \
    && ssh-keyscan -H github.com >> ~/.ssh/known_hosts

# Install python packages
RUN pip3 install numba scikit-learn networkx tqdm opencv-python

# Build dependencies
# PX4 prerequisites
RUN apt-get update \
    && apt-get install -y --no-install-recommends  \
    protobuf-compiler \ 
    libeigen3-dev \
    libopencv-dev \
    libdw-dev \
    xterm \
    python3-tk \
    python3-scipy \
    libgstreamer-plugins-base1.0-dev \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    ros-${ROS_DISTRO}-mavros-msgs \
    geographiclib-tools \
    libgeographic-dev \
    libgeographic19 \
    ros-noetic-geographic-msgs \
    ros-noetic-libmavconn \
    ros-noetic-mavlink \
    ros-noetic-uuid-msgs \
    && wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && bash ./install_geographiclib_datasets.sh \
    && rm -rf /var/lib/apt/lists/*

# Output bash terminal
CMD ["bash"]
