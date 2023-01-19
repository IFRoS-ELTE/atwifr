# ATWIFR - Automatic Tree Watering Intelligent Field Robot
Seva's (on the left) and Enrique's (on the right) Tree Inspection Robot. <br/>
![alt text](https://github.com/IFRoS-ELTE/atwifr/blob/main/media/Enrique_Seva.png?raw=true) <br/>
The project is a final project for the [IFROS](https://ifrosmaster.org/) third semester's subject "IFRoS Lab" in ELTE University, Budapest.
## Dependencies
- Follow [README](https://github.com/IFRoS-ELTE/atwifr/blob/main/atwifr_gazebo/README.md) from artwifr_gazebo to use robot in simulation;
- Install [kiss_icp_ros](https://github.com/sevagul/kiss-icp-ros/blob/main/README.md) package for exploration and mapping:
```
cd ~/catkin_ws/src
git clone https://github.com/sevagul/kiss-icp-ros.git
```
Follow it's [README](https://github.com/sevagul/kiss-icp-ros/blob/main/README.md) to install it's dependencies, afterwards build the workspace.<br/>
The kiss-icp may be configured in the `param/icp_params.yaml` file.
- Install [velodyne_height_map](https://github.com/jack-oquin/velodyne_height_map) package for mapping;
- Install [frontier_exploration](https://github.com/nocoinman/frontier_exploration) package for exploration;

To install all the dependencies, use

```
cd <your/workspace/path>
rosdep install --from-paths src --ignore-src -r -y
```
And finally build everything using `catkin build` or `catkin_make`
## Usage
### Simulation
To run the simulation with the groundtruth odometry:
```
roslaunch atwifr_bringup simulation.launch
```
To run the simulation with the kiss-icp odometry:
```
roslaunch atwifr_bringup sim_with_icp.launch
```
### Mapping
Before launching mapping, be sure that all the odometry transformations are being broadcasted. <br/>
After simulation/robot is running, launch mapping with
```
roslaunch atwifr_bringup octomap.launch
```
To manually navigate the robot, run
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
### Navigation
To setup the navigation, run
```
roslaunch atwifr_bringup navigation.launch
```
You can specify a waypoint from rviz to test how it works <br/>

If you want to run it with kiss-icp, you may need to run the following before:
```
rosrun atwifr_bringup odom_from_tf
```
This node will generate odometry messages from the /tf provided by the kiss-icp node. It uses some filtering so that the speed provided in the odometry is a bit smoothed. <br/>
Other odometry nodes may broadcast odometry messages themselves, so there might be no need to run it.

### Exploration
For exploration, run the following:
```
roslaunch frontier_exploration explore_costmap.launch costmap:=projected_map
```

### Inspection
To run the inspection, run
```
rosrun atwifr_bringup tsp
```
and call the service when ready to start inspection as the following:
```
rosservice call /plan_tsp
```
Note, that for now inspection works best when simulation is launched with groundtruth instead of kiss-icp. <br/>



### TREE-SLAM

This module is not integrated with the ones above, and can be read, installed and used from the [following repository](https://github.com/enriquea52/Tree-SLAM)

## Examples of Execution

### Mapping/Navigation/Inspection
In the next figure, you can see the inspection task in progress <br/>
![alt text](https://github.com/IFRoS-ELTE/atwifr/blob/main/media/tsp.png?raw=true)<br/>
The inspection points are marked with the red boxes, the tsp path in drawn with the dark line <br/>
You can see a video of the inspection process [here](https://youtu.be/REcBNsOYybI)
### Mapping/Navigation/Exploration/ICP
In the following Figure you can see the explored map (on the left) and the exploration in progress (on the right): <br/>
![alt text](https://github.com/IFRoS-ELTE/atwifr/blob/main/media/exploration.png?raw=true)<br/>
The detected frontier is marked with the blue color <br/>
And here is a [small demo video](https://youtu.be/jHadogrUxD4) <br/>
KISS-ICP odometry was used to produce these results. <br/>
You can also see the example of the pointclouds aligned by the kiss-icp in [kiss-icp-ros repository](https://github.com/sevagul/kiss-icp-ros)
### TREE SLAM
The examples of TREE SLAM execution can be seen in the [TREE SLAM repository](https://github.com/enriquea52/Tree-SLAM)
