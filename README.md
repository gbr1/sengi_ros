# sengi_ros
ROS package to get Sengi working with ros.

## Setup
Go into your workspace source folder (e.g. catkin_ws/src): <br>
`cd ~/catkin_ws/src` <br>
clone this repository: <br>
`git clone http://github.com/gbr1/sengi_ros.git` <br>
return in catkin_ws: <br>
`cd ..` <br>
check dependencies: <br>
`rosdep install --from-paths src --ignore-src -r -y` <br>
make: <br>
`catkin_make` <br>
install: <br>
`catkin_make install` <br>

## Launch
To launch sengi_node: <br>
`roslaunch sengi_ros sengi_node.launch` <br>
Node opens **/dev/ttyACM0** at a baudrate of **115200**. <br>

### Subcribed Topics
* _/sengi/feedback_ , speed from motors
* _/cmd_vel_ , twist from robot control 

### Published Topics
* _/sengi/cmd_drive_ , motors control
* _/sengi/status_ , battery level



## Known issues
There is no config file, so it is difficult to change parameters.  Everything works on Erwhi Hedgehog but could be useful to set parameters in a yaml file.

__Copyright (c) 2019 Giovanni di Dio Bruno under MIT license__