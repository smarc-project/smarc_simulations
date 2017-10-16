cmake_minimum_required(VERSION 2.8.3)
project(underwater_envs)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(gazebo REQUIRED) 

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uuv_descriptions
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# Install python scripts
catkin_install_python(PROGRAMS scripts/spawn_model.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY launch Media worlds models world_models
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
