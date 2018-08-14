# egh450_navigation_interface
 An example ROS package to perform high-level navigation tasks by interfacing with contrail 

## Download & Compile
```sh
cd ~/catkin_ws/src
git clone https://github.com/qutas/egh450_navigation_interface
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```


## Usage
To run a basic demonstration off of contrail navigation interface, first launch `uavusr_emulator` guidance example as per usual. Next, you will need to make a copy of the demo launch file:
```
mkdir -p ~/catkin_ws/launch
cd ~/catkin_ws/launch
roscp egh450_navigation_interface navigator_demo.launch ./
```

Next, edit the newly created launch file, using the `remap` roslaunch commands to connect the navigator interfaces with the contrail interfaces (identify them using the `rostopic list`):
```
nano ~/catkin_ws/launch/navigator_demo.launch
```

Once ready, save and run the launch file with the command:
```sh
roslaunch ~/catkin_ws/launch/navigator_demo.launch
```

Notes:
- For the best demonstration, restart the guidance node so it starts tracking waypoints. Mid-way through the waypoint tracking, run the command `rostopic pub /imagery_trigger std_msgs/Empty "{}"`