Sam control input
=================

Publish on these topics to set the fin angle or thrust.

* `/sam_auv/joint1_position_controller/command` - vector left/right
* `/sam_auv/joint2_position_controller/command` - vector up/down
* `/sam_auv/thrusters/0/input` - first thruster
* `/sam_auv/thrusters/1/input` - second thruster

Sam output values
=================

These commands give you the actual values achieved (or error, in the case of the angle commands).

* `/sam_auv/joint1_position_controller/state` - vector left/right
* `/sam_auv/joint2_position_controller/state` - vector up/down
* `/sam_auv/thrusters/0/thrust` - first thruster
* `/sam_auv/thrusters/1/thrust` - second thruster

