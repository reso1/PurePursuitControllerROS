#!/usr/bin/env python

import rospy

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker

class TrajectoryDrawer():
    def __init__(self):
        rospy.init_node('trajectory_drawer')

        frequency = rospy.get_param(
            '/trajectory_drawer/frequency', 1.0)
        odom_topic_name = rospy.get_param(
            '/trajectory_drawer/odom_topic_name', '/odometry/filtered_map')
        traj_topic_name =  rospy.get_param(
            '/trajectory_drawer/traj_topic_name', '/planned_trajectory')

        self.odom_sub = rospy.Subscriber(odom_topic_name, Odometry, self.odom_callback)
        self.traj_pub = rospy.Publisher(traj_topic_name, Path, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0/frequency), self.timer_callback)

        self.cur_pose = Pose()
        self.trajectory = Path()
        self.trajectory.header.frame_id = 'map'


    def spin(self):
        rospy.spin()

    def odom_callback(self, msg):
        self.cur_pose = msg.pose.pose

    def timer_callback(self, event):
        ps = PoseStamped()
        ps.pose = self.cur_pose
        self.trajectory.poses.append(ps)
        self.traj_pub.publish(self.trajectory)


if __name__ == '__main__':
    try:
        traj_drawer = TrajectoryDrawer()
        traj_drawer.spin()

    except rospy.ROSInterruptException:
        rospy.logerr('something went wrong.')
