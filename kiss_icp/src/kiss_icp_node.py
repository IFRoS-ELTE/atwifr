#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import os
import datetime
from functools import reduce
import os
from pathlib import Path
import sys
import time
from typing import List
from pathlib import Path
import numpy as np
from tqdm import trange
from scipy.spatial.transform import Rotation
import tf

from kiss_icp.config import KISSConfig, load_config, write_config
from kiss_icp.metrics import absolute_trajectory_error, sequence_error
from kiss_icp.odometry import Odometry
from kiss_icp.visualizer import RegistrationVisualizer, StubVisualizer
from geometry_msgs.msg import Pose, PoseStamped

import rospkg
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('kiss_icp')


class FakeData:
    def __init__(self, max_range=100):
        self.max_range = max_range

class MyOdometryPipeline:
    def __init__(
        self, # dataset, # config: Path,
        deskew: bool = False,
        visualize: bool = False
    ):
        config = Path(str(pkg_path) + "/conf/default.yaml")
        # self._dataset = dataset
        self.config: KISSConfig = load_config(config)
        data = FakeData(100)
        self.config.data = data  # TODO(Nacho): Fix this config mess

        self.odometry = Odometry(config=self.config, deskew=deskew)
        self.times = []
        self.poses = self.odometry.poses
        self.pub = rospy.Publisher('estimated_pose', PoseStamped, queue_size=1)
        self.cloud_pub = rospy.Publisher('velodyne_pcl', PointCloud2, queue_size=1)
        self.br = tf.TransformBroadcaster()

        # Visualizer
        # self.visualizer = RegistrationVisualizer() if visualize else StubVisualizer()
        # # if hasattr(self._dataset, "use_global_visualizer"):
        # self.visualizer.global_view = True#self._dataset.use_global_visualizer


    def points_callback(self, msg: PointCloud2):
        frame = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        print()
    
        timestamps = np.zeros(frame.shape[0])
        start_time = time.perf_counter_ns()
        in_frame, source = self.odometry.register_frame(frame, timestamps)
        self.times.append(time.perf_counter_ns() - start_time)
        self.publish_pose(self.poses[-1])
        msg.header.frame_id = "base_link_estimate"
        msg.header.stamp = rospy.Time.now()
        self.cloud_pub.publish(msg)

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


def talker():
    rospy.init_node('subscribe', anonymous=True)
    pipeline = MyOdometryPipeline(False, True)
    sub = rospy.Subscriber('/velodyne_points', PointCloud2, pipeline.points_callback, queue_size=10)
    rospy.spin()
    
def points_callback(msg: PointCloud2):
    pc_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
    
    rospy.loginfo(f"got msg {pc_array}")
 
if __name__ == '__main__':
    try:
        talker()     
    except rospy.ROSInterruptException:
        pass