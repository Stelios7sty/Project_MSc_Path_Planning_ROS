#!/usr/bin/env python

from __future__ import print_function
import sys
import math
import numpy as np

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class follow_the_gap():

    def __init__(self):
        #Topics & Publishers,Subscriptions
        lidar_scan_topic = '/scan'
        drive_topic = '/drive'

        self.lidar_sub = rospy.Subscriber(lidar_scan_topic, LaserScan, self.callback, queue_size = 1) 
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 1)
        self.angles = None
        self.angles_init = False
        
    def preprocess_lidar(self, data, window_size = 7):
    
        #initialize data
        steering_viewport = 70 #degrees
        ranges = np.array(data.ranges)
        if not self.angles_init:
            min_angle = data.angle_min
            max_angle = data.angle_max
            self.angles = np.linspace(min_angle, max_angle, ranges.shape[0])
            self.angles_init = True
            self.good_angle_idx = np.where(np.logical_and(self.angles > np.radians(-steering_viewport), self.angles < np.radians(steering_viewport)))
            self.angles = self.angles[self.good_angle_idx]


        ranges[np.isnan(ranges)] = 0
        goodIndex = np.isfinite(ranges)
        #do not remove all gaps
        ranges[~goodIndex] = 5

        kernel = np.ones((1,window_size))/window_size
        kernel = kernel[0,:]
        smoothed_ranges =np.convolve(ranges,kernel,'same')
        smoothed_ranges[~goodIndex] = 0
        smoothed_ranges = np.clip(smoothed_ranges, 0, 3)
        smoothed_ranges = smoothed_ranges[self.good_angle_idx]
        return smoothed_ranges

    def find_the_max_gap(self, free_space_ranges):
        #finding the size of the gaps in the simulation
        
        temp_arr = np.copy(free_space_ranges)
        temp_arr[np.nonzero(temp_arr)] = 2
        split = np.split(np.arange(free_space_ranges.shape[0]), np.where(np.abs(np.diff(temp_arr)) >= 1)[0]+1)
        
        sorted_split = sorted(split, key=len, reverse=True)
        for i in range(len(sorted_split)):
            if np.any(free_space_ranges[sorted_split[i]]):
                return np.min(sorted_split[i]), np.max(sorted_split[i]+1)
    

    
    def find_best_point(self, start_i, end_i, ranges):
        # find the suitable-largest gap and choose that one 
        return self.angles[np.argmax(ranges[start_i:end_i])+start_i]
        
    
    def set_bubble_radius(self, ranges, closest_point_idx, rb = 0.6):
        #scanning certain circle around the vehicle
        angle = self.angles[closest_point_idx]
        dtheta = np.arctan2(rb, ranges[closest_point_idx])

        bubble_idx = np.where(np.logical_and(self.angles > angle-dtheta, self.angles < angle+dtheta))

        ranges[bubble_idx] = 0

        return ranges

    def callback(self, data):
        proc_ranges = self.preprocess_lidar(data)

        #Find closest point to LiDAR
        closest_point_idx = np.argmin(proc_ranges[np.nonzero(proc_ranges)])
       


        #set all the points to zero in the selected circle
        bubbled_ranges = self.set_bubble_radius(proc_ranges, closest_point_idx)

        #Find max length in the gap
        gap_start, gap_end = self.find_the_max_gap(bubbled_ranges)
        

        #Choose the best point in the gap 
        follow_this_angle = self.find_best_point(gap_start, gap_end,bubbled_ranges)

    
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = follow_this_angle
        drive_msg.drive.speed = 0.50 #slow constant velocity for now
        rospy.loginfo("Follow this angle {}".format(follow_this_angle))

        self.drive_pub.publish(drive_msg)




def main(args):
    rospy.init_node("Follow_The_Gap", anonymous=True)
    ftg = follow_the_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)