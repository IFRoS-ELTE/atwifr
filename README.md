# ATWIFR - Automatic Tree Watering Intelligent Field Robot
Enrique's and Seva's Watering Robot.
## Dependencies
- Follow [README](https://github.com/IFRoS-ELTE/atwifr/blob/main/atwifr_gazebo/README.md) from artwifr_gazebo;
- Install kiss_icp package with pip:
```
pip3 install kiss-icp scipy 
```
If fail, try to follow the original kiss-icp [README](https://github.com/PRBonn/kiss-icp), update pip, etc. <br/>

## Usage
To run the two packages together:
```
roslaunch atwifr_gazebo launch_with_icp.launch
```
This would work only if you followed full installation for both packages. <br/>
To learn about each package usage, follow their README file.

