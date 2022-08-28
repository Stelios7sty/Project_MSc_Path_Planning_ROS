#!/usr/bin/env python

from itertools import count
from nav_msgs.msg import Path
import rospy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np 
import time
from race.msg import drive_param
import os
import rospkg
import atexit
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class lab_time():
    def __init__(self):
        odom_topic = "/odom"
        driver_topic = "/drive"
        path_topic = "/smoothed_path"
        self.odometry_sub=Subscriber(odom_topic, Odometry)
        self.drive_sub=Subscriber(driver_topic,AckermannDriveStamped)
        self.path_sub = Subscriber(path_topic,Path)
        self.sub = ApproximateTimeSynchronizer([self.odometry_sub,self.drive_sub], queue_size = 20, slop = 0.05)

        self.sub.registerCallback(self.master_callback)
        self.startTime = 0
        self.count = 0


       
        self.times=[]

        self.start_time=time.time()

    def master_callback(self,odom_msg,ackermann_msg):

        if(self.count == 0):
            self.startTime = rospy.Time.now()
            time = 0 
        else:
            time = rospy.Time.now()- self.startTime
            time = np.round(time.to_sec(),decimals=1)
        
        if(time == 76.79):
            rospy.logwarn("Lab completed")
            self.reset()

       
        print("time for the labs:",time)

        if(self.count>0):
            self.times.append(time)

        self.count+=1
    
    def reset(self):
        self.times=[]
        self.count = -1
        



def main():
    rospy.init_node('lab_time')
    pp = lab_time()
    rospy.spin()
if __name__ == '__main__':
    main()