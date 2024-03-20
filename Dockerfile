# Base image
FROM nvidia/cuda:12.1.1-devel-ubuntu20.04
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

## ========== THESIS =============

# Create the workspace
RUN mkdir -p ~/thesis_ws/src \
    && cd ~/thesis_ws \
    && catkin init \
    && catkin config --extend ${ROS_ROOT} 

# clone and update main repo
RUN --mount=type=ssh cd ~/thesis_ws/src \
    && git clone git@github.com:nikhil-sethi/thesis.git -b docker_move \
    && cd ~/thesis_ws/src/thesis \ 
    && git submodule update --init --recursive  # might take some time

# Build third party sw
# PX4 prerequisites
RUN apt-get update \
    && apt-get install -y --no-install-recommends  \
    	protobuf-compiler \ 
    	libeigen3-dev \
    	libopencv-dev \
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
    
RUN cd ~/thesis_ws/src/thesis/sw/simulation/PX4-Autopilot/ \
    && bash ./Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx 

# add this to prereqs above
RUN apt-get install -y --no-install-recommends libgstreamer-plugins-base1.0-dev

# PX4 build
RUN cd ~/thesis_ws/src/thesis/sw/simulation/PX4-Autopilot/ \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && DONT_RUN=1 make -j10 px4_sitl_default gazebo_iris_vision


# FUEL prereqs
# NLOPT
RUN cd ~/thesis_ws/src/thesis/sw/third_party/nlopt \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install # will install the main library in /usr/local/lib

# extra libs 
RUN apt-get update \ 
    && apt-get install -y --no-install-recommends \
    libdw-dev \ 
    xterm \
    python3-tk \
    python3-scipy \
    && rm -rf /var/lib/apt/lists/*

# build everything
RUN cd ~/thesis_ws \ 
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && catkin config --skiplist hovergames_control hovergames_mpc_identification hovergames_sim_identification hovergames_flight_identification testbench_identification hovergames_mpc_model_mismatch minimal px4 rviz_plugins multi_map_server\
    && catkin build
    
# fake stop
# RUN fake_stop

# Source ROS and workspace
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source ~/thesis_ws/devel/setup.bash" >> ~/.bashrc


# Output bash terminal
CMD ["bash"]
