#!/usr/bin/env python3

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
pkg_path = Path(rospack.get_path('kiss_icp'))


# Main class
class KissIcpOdometry:
    def __init__(self, config: str = None):
        
        # read config
        if config is None:
            config = Path(pkg_path, "config", "default.yaml") # default config
        else:
            config = Path(config) # passed in object
        config = rospy.get_param("~config", config) # passed as ros parameter (highest priority)
        self.config: KISSConfig = load_config(config)
        # define default value for parameters, added to config manually (originally wasn't there)
        deskew = False
        if hasattr(self.config, "deskew"):
            deskew = self.config.deskew
        
        # define used objects
        self.odometry = Odometry(config=self.config, deskew=deskew)
        self.times = []
        self.poses = self.odometry.poses

        # define frames and pubs, subs
        self.frame_id_estimate = "base_link"
        self.frame_id_global = "odom"
        if hasattr(self.config, "frames") and hasattr(self.config.frames, "estimate"):
            self.frame_id_estimate = self.config.frames.estimate
        if hasattr(self.config, "frames") and hasattr(self.config.frames, "global_frame"):
            self.frame_id_global = self.config.frames.global_frame

        self.pub = rospy.Publisher('~estimated_pose', PoseStamped, queue_size=1)
        self.points_pub = rospy.Publisher( 'velodyne_pcl', PointCloud2, queue_size=1)
        self.points_sub = rospy.Subscriber('/points_in', PointCloud2, self.points_callback, queue_size=10)
        
        self.br = tf.TransformBroadcaster()


    def points_callback(self, msg: PointCloud2):
        frame = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        timestamps = np.zeros(frame.shape[0])
        start_time = time.perf_counter_ns()
        in_frame, source = self.odometry.register_frame(frame, timestamps)
        self.times.append(time.perf_counter_ns() - start_time)
        self.publish_pose(self.poses[-1])
        msg.header.frame_id = self.frame_id_estimate
        msg.header.stamp = rospy.Time.now()
        self.points_pub.publish(msg)

    def publish_pose(self, pose):
        R_array = pose[:3, :3]
        R = Rotation.from_matrix(R_array)
        R_q = R.as_quat()
        t = pose[:3, 3]
        p = PoseStamped()
        p.header.frame_id = self.frame_id_global
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
                        self.frame_id_estimate,
                        self.frame_id_global)
 
if __name__ == '__main__':
    rospy.init_node('kiss_icp')
    odometry = KissIcpOdometry()
    rospy.spin()