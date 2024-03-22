 
this_script_path=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)

 export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$this_script_path/sw/simulation/PX4-Autopilot/:$this_script_path/sw/simulation/PX4-Autopilot/Tools/sitl_gazebo/
 source $this_script_path/sw/simulation/PX4-Autopilot/Tools/setup_gazebo.bash $this_script_path/sw/simulation/PX4-Autopilot $this_script_path/sw/simulation/PX4-Autopilot/build/px4_sitl_default