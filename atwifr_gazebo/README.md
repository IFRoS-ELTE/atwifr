# Custom Simulation for IFROS ELTE Project

## Dependencies

UGV SDK for controlling the robot
```
cd <your/ws/path>/src
git clone git@github.com:agilexrobotics/ugv_gazebo_sim.git
```
And install dependencies for it:
```
sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control ros-noetic-joint-state-publisher-gui ros-noetic-teleop-twist-keyboard 
```
Intel Realsense Packages:
```
git clone https://github.com/issaiass/realsense2_description
git clone https://github.com/issaiass/realsense_gazebo_plugin
git clone https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/
```

Install packages dependencies:
```
cd <your/ws/path>
rosdep install --from-paths src --ignore-src -r -y
```

## Usage

To run the simulation:

```
roslaunch atwifr_gazebo scout_ifros_world.launch
```
To teleoperate:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```