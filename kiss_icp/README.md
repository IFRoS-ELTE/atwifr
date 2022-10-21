# KISS-ICP ros node

## Dependencies
```
pip3 install kiss-icp scipy
```
If fail, try to follow the original kiss-icp [README](https://github.com/PRBonn/kiss-icp), update pip, etc. <br/>

## Usage:
To run kiss-icp, run
```
roslaunch kiss_icp run_icp.launch [config:=<your config>] [points_in_topic:=<your point cloud topic>]
```
To run it on the bag-file, run
```
roslaunch kiss_icp run_icp_bag.launch [bagfile:=<your bagfile>] [config:=<your config>] [points_in_topic:=<your point cloud topic>]
```