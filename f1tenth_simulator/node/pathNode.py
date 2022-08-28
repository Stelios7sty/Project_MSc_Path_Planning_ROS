#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry, Path

import numpy as np
# TODO: import ROS msg types and libraries

class PathNode(object):
  
    def __init__(self):
        
        
        self.waypoints = np.genfromtxt('/home/stelios/catkin_ws/src/f1tenth_simulator/waypoints/waypoints_levine_2.csv',delimiter=',')
        self.waypoints = self.waypoints[:,0:2]



        self.path_msg = Path()
        self.path_msg.header.stamp=rospy.Time.now()
        self.path_msg.header.frame_id = "map"
        
        for i in range(self.waypoints.shape[0]):
            loc = PoseStamped()
            loc.header.stamp = rospy.Time.now()
            loc.header.frame_id = "map"
            loc.pose.position.x = self.waypoints[i, 0]
            loc.pose.position.y = self.waypoints[i,1]
            loc.pose.position.z = 0
            self.path_msg.poses.append(loc)
        
        pf_odom_topic = '/odom'
        path_topic = '/smoothed_path'
        self.odom_sub = rospy.Subscriber(pf_odom_topic, Odometry, self.pose_callback, queue_size = 1)
        self.path_pub = rospy.Publisher(path_topic, Path, queue_size = 1)



    def pose_callback(self, pose_msg):

        self.path_pub.publish(self.path_msg)




def main():
    rospy.init_node('path_publisher')
    pp = PathNode()
    rospy.spin()
if __name__ == '__main__':
    main()