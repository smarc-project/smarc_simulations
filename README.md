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

To be continued...
