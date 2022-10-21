#!/usr/bin/env python

# python libs
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation

# kiss_icp libs
from kiss_icp.config import KISSConfig, load_config
from kiss_icp.odometry import Odometry

# ros libs
import rospy
import ros_numpy
import rospkg

# ros msgs libs
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import tf

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('kiss_icp')

# Class to fit node to the existing kiss_icp.odometry.Odometry class
class FakeData:
    def __init__(self, max_range=100):
        self.max_range = max_range

# Main class
class KissIcpOdometry:
    def __init__(self, deskew: bool = False, config: str = None):
        if config is None:
            config = Path(str(pkg_path) + "/config/default.yaml")
        else:
            config = Path(config)
        self.config: KISSConfig = load_config(config)
        data = FakeData(100)
        self.config.data = data  # TODO(Nacho): Fix this config mess

        self.odometry = Odometry(config=self.config, deskew=deskew)
        self.times = []
        self.poses = self.odometry.poses

        self.pub = rospy.Publisher('estimated_pose', PoseStamped, queue_size=1)
        self.points_pub = rospy.Publisher( 'velodyne_pcl', PointCloud2, queue_size=1)
        self.points_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.points_callback, queue_size=10)
        
        self.br = tf.TransformBroadcaster()


    def points_callback(self, msg: PointCloud2):
        frame = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        timestamps = np.zeros(frame.shape[0])
        start_time = time.perf_counter_ns()
        in_frame, source = self.odometry.register_frame(frame, timestamps)
        self.times.append(time.perf_counter_ns() - start_time)
        self.publish_pose(self.poses[-1])
        msg.header.frame_id = "base_link_estimate"
        msg.header.stamp = rospy.Time.now()
        self.points_pub.publish(msg)

    def publish_pose(self, pose):
        R_array = pose[:3, :3]
        R = Rotation.from_matrix(R_array)
        R_q = R.as_quat()
        t = pose[:3, 3]
        p = PoseStamped()
        p.header.frame_id = "world"
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = t[0]
        p.pose.position.y = t[1]
        p.pose.position.z = t[2]
        p.pose.orientation.x = R_q[0]
        p.pose.orientation.y = R_q[1]
        p.pose.orientation.z = R_q[2]
        p.pose.orientation.w = R_q[3]
        self.pub.publish(p)

        self.br.sendTransform((t[0], t[1], t[2]),
                        R_q,
                        rospy.Time.now(),
                        "base_link_estimate",
                        "world")
 
if __name__ == '__main__':
    try:
        rospy.init_node('kiss_icp', anonymous=True)
        odometry = KissIcpOdometry(deskew=False)
        rospy.spin()     
    except rospy.ROSInterruptException:
        pass