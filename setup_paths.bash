 
this_script_path=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)
# custom models and worlds
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$this_script_path/sw/bringup/models

 export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$this_script_path/sw/simulation/PX4-Autopilot/:$this_script_path/sw/simulation/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/
 source $this_script_path/sw/simulation/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash $this_script_path/sw/simulation/PX4-Autopilot $this_script_path/sw/simulation/PX4-Autopilot/build/px4_sitl_default