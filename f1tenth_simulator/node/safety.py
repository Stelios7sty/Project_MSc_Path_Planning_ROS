#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

class Safety(object):

    def __init__(self):
        """
        safety node to publish in the brake and brake bool
        in order to help the Follow the gap algorithm 
        to perform better when it is near to obstacles
        """
        self.speed = 0
        self.TTC_thresh = 0.3 #1 second TTC threshold

        self.init_publishers()
        self.init_subscribers()

    def init_publishers(self):
        self._brake_publisher = rospy.Publisher("/brake", AckermannDriveStamped, queue_size = 1)
        self._brake_bool_publisher = rospy.Publisher("/brake_bool", Bool, queue_size = 1)

    def init_subscribers(self):
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size = 1)
        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size = 1)


    def odom_callback(self, odom_msg):
        # update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        #  calculate TTC
        #first, grab ranges and generate angle vector
        if self.speed != 0:
            ranges = np.array(scan_msg.ranges)
            min_angle = scan_msg.angle_min
            max_angle = scan_msg.angle_max
            increment = scan_msg.angle_increment
            angle_vec = np.arange(min_angle, max_angle, increment)
            dr = np.cos(angle_vec)*self.speed
            denom = np.where(dr < 0, 0.01, dr)
            TTC = ranges/denom
            rospy.loginfo("Min Range {}, TTC {}".format(np.min(ranges), np.min(TTC)))

            #Publish brake bool and brake message in emergency
            if np.any(TTC < self.TTC_thresh):
                ack_msg = AckermannDriveStamped()
                ack_msg.drive.speed = 0.0
                brake_bool = Bool()
                brake_bool.data = True
                self._brake_publisher.publish(ack_msg)
                self._brake_bool_publisher.publish(brake_bool)
        else:
            brake_bool = Bool()
            brake_bool.data = False
            self._brake_bool_publisher.publish(brake_bool)





def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()