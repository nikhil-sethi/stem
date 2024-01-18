workspace setup
```
cd thesis_ws
source ~/racer_ws/devel/setup.bash
catkin build
source devel/setup.bash
export px4_path=/root/software/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_path/:$px4_path/Tools/simulation/gazebo-classic/sitl_gazebo-classic/
source $px4_path/Tools/simulation/gazebo-classic/setup_gazebo.bash $px4_path $px4_path/build/px4_sitl_default
```