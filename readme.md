
## Clone and setup
```bash
mkdir thesis_ws
cd thesis_ws
git clone git@github.com:nikhil-sethi/thesis.git
cd thesis
git submodule update --init --recursive  # might take some time
```

## Build
(Remove sudo if you're on docker priveliged mode)

1. PX4
```bash
cd sw/simulation/PX4-Autopilot/
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh   
```
At this point you can (and probably should) test PX4 SITL if you want before moving on.

2. NLopt (because ros-nlopt doesn't work with FUEL)
```bash
cd sw/third_party/nlopt
mkdir build
cd build
cmake ..
make
make install # will install the main library in /usr/local/lib
```

3. Main code
```bash
cd thesis_ws
caktin config --skiplist hovergames_control hovergames_mpc_identification hovergames_sim_identification hovergames_flight_identification testbench_identification hovergames_mpc_model_mismatch minimal px4
catkin build
source devel/setup.bash # standard ros source
source src/thesis/setup_paths.bash  # need this for px4 sitl and gazebo
```

## Run

Simulation:
```bash
terminal 1: roslaunch bringup main.launch vehicle:=iris_depth_camera sim:=true

terminal 2: roslaunch bringup rviz.launch

terminal 3 (for now): rosrun attention_map attention_processor_node
```
If everything went well you should see an axis marker for the drone and camera in Rviz along with occupancy map. If not, go to the debugging section.


Hardware testing

- Your PC
```bash
terminal 1: roslaunch bringup main.launch sim:=false
terminal 2: roslaunch bringup rviz.launch
```

- OBC (Nvidia Jetson Xavier)
(Assuming packages are sourced, built and everything)
```bash

terminal 1: roslaunch vicon_bridge mantis.launch
terminal 2: roslaunch realsense2_camera rs.launch
terminal 3: roslaunch px4_tools px4_tools.launch sim:=false
```


## Debugging

1. **I don't see the odometry at the mavros/local_position/odom topic**
- Check remote configuration if valid. Both ROS_MASTER_URI and ROS_IP should be set to valid values
- 

2. **I don't see the odomtery in rviz but i can see it in rostopic echo.** 
- Rviz on docker is particular about whether or not it initiated the master. So TF frames don't come sometimes. To solve it either just restart rviz once or make sure that the ros master starts up with rviz. This solves it always.

3. **I don't see the camera sensor topic /camera/pose or it's axis in rviz.**
- this means that the sw/planning/racer_if.py file failed to send or recieve transforms. Check the tf tree if everything is up and running and connected.

4. **I see the sensor pose and odometry but i don't see the occupancy map**
- There could be many reasons but as a first check the following topics if they're populated
    1. depth image topic. (/iris_depth_camera/camera/depth/image_rect_raw) or anything that you expect to recieve on.
    2. odometry topic: mavros/local_position/odom
    3. sensor pose topic: /camera/pose
- If you're on a remote networking setup, check if the unix time is synced between the robot and your pc. The command: `date +%s`. If not then follow [these](https://github.com/cor-drone-dev/mantis-3-drone/blob/main/ros_multi_machine.md#time-synchronization) instructions to do it.
- check these variables in the fuel parameters given in algorithm.xml and see if they make sense and work with each other
    - map_size_x
    - map_size_y
    - map_size_z
    - sdf_map/ground_height
    - sdf_map/box_min_x
    - sdf_map/box_min_y
    - sdf_map/box_min_z
    - sdf_map/box_max_x
    - sdf_map/box_max_y
    - sdf_map/box_max_z

4. **I get the error that the package 'px4' wasn't found**
- the setup_paths.bash script failed for some reason. Basically the script sets the internal path of the sitl_gazebo-classic ros package, and some environment variables. with some digging you can easily find how to do this manually for your setup as well.

5. **I get a segmentation fault in the exploration node**
- You are at god's own mercy. best of luck

6. **While hardware testing, i get the error 'Global data lost for over half a second'**
- There is something up with either the wifi router at the lab or time syncing between your pc and the drone OBC. 
- For time syncing check the link mentioned before. 
- For wifi router, ping the jetson from your pc and see if is under 20-30 ms. If you see very high and erratic ping times like 200ms, check issue #__ (TODO add issue for realsense pwr mgmt here). 

7. **On a vscode remote docker container, I run roscore but nothing happens**
- Check issue #13
