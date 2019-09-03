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

Next, edit the newly created launch file, using the `remap` roslaunch commands to connect the navigator interfaces with the contrail interfaces (the given example will already work for the `uavusr_emulator`):
```
nano ~/catkin_ws/launch/navigator_demo.launch
```

Once ready, save and run the launch file with the command:
```sh
roslaunch ~/catkin_ws/launch/navigator_demo.launch
```
This will begin send a path to contrail to track from `[0.0,0.0,1.0]` to `[3.0,0.0,1.0]` (hardcoded in the `src/egh450_navigation_interface/navigation_interface.py` file). Mid-way through the path, run the following command:
```sh
rostopic pub -1 /imagery_trigger std_msgs/Empty "{}"
```
This will send the trigger message to the navigator, where the message is an empty trigger. You should see the uav stop where it is, wait for 5 seconds, then resume the path.
