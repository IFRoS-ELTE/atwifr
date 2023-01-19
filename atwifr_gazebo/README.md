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

Install gazebo repo with a tree model: https://github.com/tudelft/gazebo_models. Follow it's  README to add trees to your GAZEBO PATH. 

Intel Realsense Packages following README files:
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
And finally build everything using `catkin build` or `catkin_make`

## Usage

To run the simulation and spaw a robot:

```
roslaunch atwifr_gazebo world.launch
roslaunch atwifr_gazebo spawn_scout_v2.launch
```
To teleoperate:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```