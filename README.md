# atwifr
Enrique and Seva Watering Robot
- Follow README from artwifr_gazebo
- install kiss_icp package with pip:
```
pip3 install kiss-icp scipy 
```
If fail, follow the [original Readme](https://github.com/PRBonn/kiss-icp), try to update pip, etc. <br/>

To run the two packages together:
```
roslaunch atwifr_gazebo launch_with_icp.launch
```
This would work only if you followed full installation for both packages.

