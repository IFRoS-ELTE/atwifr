# ATWIFR - Automatic Tree Watering Intelligent Field Robot
Enrique's and Seva's Watering Robot.
## Dependencies
- Follow [README](https://github.com/IFRoS-ELTE/atwifr/blob/main/atwifr_gazebo/README.md) from artwifr_gazebo;
- Install [kiss_icp_ros](https://github.com/sevagul/kiss-icp-ros/blob/main/README.md) package:
```
cd ~/catkin_ws/src
git clone https://github.com/sevagul/kiss-icp-ros.git
```
Follow it's [README](https://github.com/sevagul/kiss-icp-ros/blob/main/README.md) to install it's dependencies, afterwards build the workspace.

## Usage
To run the two packages together:
```
roslaunch atwifr_gazebo launch_with_icp.launch
```
This would work only if you followed full installation for both packages. <br/>
To learn about each package usage, follow their README file.

