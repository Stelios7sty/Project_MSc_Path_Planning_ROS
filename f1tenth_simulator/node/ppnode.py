#!/usr/bin/env python

import rospy

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

import math
import numpy as np
from numpy import linalg as la
import csv
import os
import rospkg 


class ppnode(object):

    def __init__(self):

       
        self.waypoint_file = "waypoints_levine_2.csv"

        # pure pursuit parameters
        self.LOOKAHEAD_DISTANCE = 2.0 #1.70 # meters
        


        self.read_the_waypoints()
       
        # Publisher for the goal point
        goal_topic = "/goal_point"
        vis_topic = "/vis_markers"
        points_topic = "/points"
        #goal_topic = "/"
        odom_topic = "/odom"
        drive_topic = "/drive" 
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 1)
        self.goal_pub = rospy.Publisher(goal_topic, MarkerArray, queue_size=5)
        self.considered_pub= rospy.Publisher(vis_topic, MarkerArray, queue_size=5)
        self.point_in_car_frame= rospy.Publisher(points_topic, MarkerArray, queue_size=5)

        # Subscriber to vehicle position 
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.callback, queue_size=1)

    # Import waypoints.csv into a list (path_points)
    def read_the_waypoints(self):

        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        #get the path for this paackage
        file_path=rospack.get_path('f1tenth_simulator')
        filename=os.path.sep.join([file_path,'waypoints',self.waypoint_file])

        with open(filename) as f:
            waypoints = [tuple(line) for line in csv.reader(f)]

        # Turn path_points into a list of floats to eliminate the need for casts in the code below.
        self.path_points_x   = np.asarray([float(point[0]) for point in waypoints])
        self.path_points_y   = np.asarray([float(point[1]) for point in waypoints])

        # list of xy pts 
        self.xy_points = np.hstack((self.path_points_x.reshape((-1,1)),self.path_points_y.reshape((-1,1)))).astype('double')
   
    def visualize_point(self,pts,frame='map', r=250.0,g=0.0,b=0.0):
        # create a marker array
        markerArray = MarkerArray()

        idx = np.random.randint(0,len(pts))
        point = pts[idx]

        x = float(point[0])
        y = float(point[1])
		
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.id = 15000
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.lifetime = rospy.Duration(0.01)
        markerArray.markers.append(marker)
        self.goal_pub.publish(markerArray)
    
    def callback(self,pose_msg):
        
        qx = pose_msg.pose.pose.orientation.x
        qy = pose_msg.pose.pose.orientation.y
        qz = pose_msg.pose.pose.orientation.z
        qw = pose_msg.pose.pose.orientation.w
       

        quaternion = (qx,qy,qz,qw)
        euler = euler_from_quaternion(quaternion)
        yaw   = np.double(euler[2])

        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y

        # finding the distance of each way point from the current position 
        location = np.asarray([x,y]).reshape((1,2))
        distance = np.linalg.norm(self.xy_points-location,axis=-1)

        #finding the points that are less than the lookahead distance
        goal_array = np.where((distance > self.LOOKAHEAD_DISTANCE) & (distance<self.LOOKAHEAD_DISTANCE+0.3))[0]
         
        pts = self.xy_points[goal_array]

        # get the points in front of the car, using the orientation 
        points_infrontofcar=[]
        for idx in range(len(pts)): 
            v1 = pts[idx] - location
            #since the euler was specified in the order x,y,z the angle is wrt to x axis
            v2 = [np.cos(yaw), np.sin(yaw)]

            angle= self.find_angle(v1,v2)
            if angle < np.pi/2:
                print("step 3")
                points_infrontofcar.append(pts[idx])

        points_infrontofcar =np.asarray(points_infrontofcar)
        # compute new distances
        distance = np.linalg.norm(points_infrontofcar-location,axis=-1)- self.LOOKAHEAD_DISTANCE
        
        # get the point closest to the lookahead distance
        idx = np.argmin(distance)

        # goal point 
        goal_point = points_infrontofcar[idx]
    

        
        # transform it into coordinates
        v1 = (goal_point - location)[0].astype('double')
        xgv = (v1[0] * np.cos(yaw)) + (v1[1] * np.sin(yaw))
        ygv = (-v1[0] * np.sin(yaw)) + (v1[1] * np.cos(yaw))


        self.visualize_point([goal_point])
        
        # calculate the steering angle
        angle = math.atan2(ygv,xgv)
       
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = self.set_speed(angle)
        self.drive_pub.publish(drive_msg)
    

    #set the speed in angles
    def set_speed(self, angle):
        abs_angle = np.abs(angle)
        if abs_angle >= 0.4:
            speed = 0.5
            self.LOOKAHEAD_DISTANCE = 0.5
        elif abs_angle < 0.4 or abs_angle >= 0.2:
            speed = 0.80
            self.LOOKAHEAD_DISTANCE = 0.75
        else:
            speed = 2
            self.LOOKAHEAD_DISTANCE =1.70
        return speed

    # find the angle bewtween the vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2).astype('double')
        sinang = la.norm(np.cross(v1, v2)).astype('double')
        return np.arctan2(sinang, cosang).astype('double')   

    
    

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_ttt')
    
    C = ppnode()  

    # spin
    rospy.spin()