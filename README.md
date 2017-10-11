# smarc_simulations
Simulation environments for smarc scenarios

# Setting up gazebo and uuv_simulator

## Install

Follow the instructions on http://gazebosim.org/tutorials?tut=install_ubuntu and
in https://github.com/uuvsimulator/uuv_simulator/wiki#using-uuv-simulator-with-ros-kinetic-and-gazebo-7 .

Then clone https://github.com/smarc-project/uuv_simulator.git in your catkin workspace.
Compile it with `catkin_make`, followed by `catkin_make install`.

## Environment variables

Add the following lines to your `.bashrc`, replace `GAZEBO_PREFIX` to be the path to your catkin workspace.
```
source /usr/share/gazebo-7/setup.sh
export GAZEBO_MODEL_DATABASE_URI=http://gazebosim.org/models
export GAZEBO_PREFIX=$HOME/path/to/your/catkin_ws/install
export GAZEBO_RESOURCE_PATH=${GAZEBO_PREFIX}/share/uuv_descriptions:${GAZEBO_RESOURCE_PATH}
export GAZEBO_MODEL_PATH=${GAZEBO_PREFIX}/share/uuv_descriptions/worlds:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=${GAZEBO_PREFIX}/lib:${GAZEBO_PREFIX}/lib/x86_64-linux-gnu:${GAZEBO_PLUGIN_PATH}
```

# Running a basic simulation

## Launching

Open a new tab, don't forget to source your catkin workspace every time you do this.

Then launch gazebo with an empty world using:
```
roslaunch uuv_descriptions empty_underwater_world.launch
```
Then we open a new tab and launch a simulation of the small smarc auv:
```
roslaunch small_smarc_auv upload_small_smarc_auv.launch
```

## Playing around

You can fire the thruster of the AUV by running the following command, feed it a value between 0-100:
```
rostopic pub /small_smarc_auv/thrusters/0/input uuv_gazebo_r_plugins_msgs/FloatStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
data: 100.0"
```
Check `rostopic list` and try to control e.g. the fins.

## Rviz

In rviz, you can get the camera image on `/small_smarc_auv/small_smarc_auv/camera/camera_image`
and the left and right side scans on `small_smarc_auv/sss_left` and `small_smarc_auv/sss_right`, respectively.

