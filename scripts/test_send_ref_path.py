#!/usr/bin/env python

import os
import sys
import math

import rospy
import rospkg

from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray

from tf.transformations import quaternion_from_euler


def build_path(wp_path):
    # read waypoints
    wps = []
    wp_file = open(wp_path)
    for line in wp_file.readlines():
        x, y = [float(i) for i in line.split(',')]
        wps.append([x, y])
    wp_file.close()

    # build path
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = rospy.Time.now()

    for idx in range(len(wps)-1):
        # calculate quaternion
        dx = math.sqrt((wps[idx+1][0]-wps[idx][0])**2)
        dy = math.sqrt((wps[idx+1][1]-wps[idx][1])**2)
        quaternion = quaternion_from_euler(0, 0, math.atan2(dy, dx))

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position = Point(wps[idx][0], wps[idx][1], 0.0)
        pose.pose.orientation = Quaternion(0, 0, quaternion[2], quaternion[3])
        
        path.poses.append(pose)
    
    rospy.loginfo('fetched ' + str(len(wps)) + ' waypoints')

    if wps == []:
        rospy.signal_shutdown('No waypoint to draw...shutdown')

    return path


def main():
    rospy.init_node('test_send_ref_path')

    ref_path_topic_name = rospy.get_param(
        '/pure_pursuit_controller/ref_path_topic_name', '/reference_path')
    ref_path_pub = rospy.Publisher(ref_path_topic_name, Path, queue_size=1)

    r = rospy.Rate(1)

    wp_path = rospy.get_param('waypoint_file_path', 
        default=os.path.join(rospkg.RosPack().get_path('pure_pursuit_controller'),'wp','path.txt'))

    path = build_path(wp_path)

    while not rospy.is_shutdown():
        ref_path_pub.publish(path)
        r.sleep()


if __name__ == "__main__":
    try:    
        main()

    except rospy.ROSInterruptException:
        rospy.logerr('Get KeyBoardInterrupt...Shutdown')