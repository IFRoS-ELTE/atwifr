while :
do
    roslaunch atwifr_bringup run_icp.launch &
	rosbag play data_prg.bag
	rosnode kill /kiss_icp/kiss_icp_node
done