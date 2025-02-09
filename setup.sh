
ws_path=/workspaces/stem_ws
repo_path=$ws_path/src/thesis

# Get the dependencies
cd $ws_path/src/thesis
mkdir -p sw/dependencies 
vcs import sw/dependencies < .repos


# ======== Build the dependencies ================
cd $repo_path/sw/dependencies

cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx 
source /opt/ros/${ROS_DISTRO}/setup.bash
DONT_RUN=1 make -j10 px4_sitl_default gazebo_iris_vision

# NLOPT build (for FUEL)
cd $ws_path/src/thesis/sw/dependencies/nlopt
mkdir build 
cd build 
cmake .. 
make 
make install # will install the main library in /usr/local/lib



# Build ROS workspace
cd $ws_path 
source /opt/ros/${ROS_DISTRO}/setup.bash 
catkin config --skiplist hovergames_control hovergames_mpc_identification hovergames_sim_identification hovergames_flight_identification testbench_identification hovergames_mpc_model_mismatch minimal px4 rviz_plugins multi_map_server odom_visualization attention_map
catkin build

# Source ROS and workspace
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc 
echo "source $ws_path/devel/setup.bash" >> ~/.bashrc 
echo "source /workspaces/src/thesis/setup_paths.bash" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$repo_path/sw/stem_bringup/models" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$repo_path/sw/dependencies/PX4-Autopilot/:$repo_path/sw/dependencies/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/" >> ~/.bashrc
echo " source $repo_path/sw/dependencies/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash $repo_path/sw/dependencies/PX4-Autopilot $repo_path/sw/dependencies/PX4-Autopilot/build/px4_sitl_default
