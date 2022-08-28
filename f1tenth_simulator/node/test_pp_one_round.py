#!/usr/bin/env python
from signal import signal
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, Pose, PolygonStamped
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray , Marker
from tf.transformations import quaternion_matrix
import numpy as np


class PurePursuit(object):

    def __init__(self):

        
        self.waypoints = np.genfromtxt('/home/stelios/catkin_ws/src/f1tenth_simulator/waypoints/waypoints_levine.csv',delimiter=',')
        self.waypoints = self.waypoints[:,0:2]
        self.waypoints = self.waypoints[0:int(self.waypoints.shape[0]*0.90),:]

        self.lookahead = 1
        
        
        odom_topic = '/odom'
        drive_topic = '/drive'
        waypoints_topic = '/waypoints'
        marker_topic = '/points'
        waypoints_topic_2 = '/point_2'
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.pose_callback, queue_size = 1)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 1)
        self.waypoint_pub = rospy.Publisher(waypoints_topic, PointStamped, queue_size = 1)
        self.points_pub = rospy.Publisher(marker_topic, MarkerArray, queue_size= 1)
        self.waypoint_pub_2 = rospy.Publisher(waypoints_topic_2, PointStamped, queue_size = 1)
        self.last_idx = 0

    def find_path(self, current_node):
        try:
            self.current_node = self.waypoints
            p = self.find_path(self.current_node[self.waypoints])
            path = []
            path.extend(p)
            path.append(current_node)
            return path
        except KeyError:
            # we have reached the start node
            return [current_node]

    def get_curr_waypoint(self, location):
        stop = False
        if self.last_idx >= self.waypoints.shape[0]:
            stop = True
            return None, stop

        dist = (self.waypoints[:,0]-location[0])**2+(self.waypoints[:,1]-location[1])**2
        for i in range(self.waypoints.shape[0]-1, self.last_idx,-1):
            if dist[i] < self.lookahead:
                self.last_idx = i

                break           
        waypoint_pre = self.waypoints[self.last_idx]
        waypoint_post = self.waypoints[self.last_idx+1]

        waypoint = (waypoint_pre+waypoint_post)/2 
        return waypoint, stop

    def set_speed(self, angle):
        abs_angle = np.abs(angle)
        if abs_angle >= 0.4:
            speed = 0.5
            self.lookahead = 0.5
        elif abs_angle < 0.4 or abs_angle >= 0.2:
            speed = 0.75
            self.lookahead = 0.75
        else:
            speed = 1
            self.lookahead =1.70
        return speed


    
    def pose_callback(self, pose_msg):
        location = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y]
        quaternion = [pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w]
        rot_matrix = quaternion_matrix(quaternion)[0:3, 0:3]
       
        target_waypoint, stop = self.get_curr_waypoint(location)

       
        if not stop:
            print("In If statement")
            #data = Odometry() 
            
            waypoint_msg = PointStamped()
            waypoint_msg.header.stamp = rospy.Time.now()
            waypoint_msg.header.frame_id = "map"
            waypoint_msg.point.x = target_waypoint[0] 
            waypoint_msg.point.y = target_waypoint[1]
            waypoint_msg.point.z = 0
            self.waypoint_pub.publish(waypoint_msg)

            # find the current waypoint to track

            #  transform goal point to vehicle frame of reference
            #  calculate curvature/steering angle
            goal_point_world = target_waypoint-location
            goal_point_world = np.append(goal_point_world, 0)
            goal_point_body = np.matmul(np.linalg.inv(rot_matrix), goal_point_world)
            #print("goal body: ",goal_point_body, "goal_world",goal_point_world)
            
            angle = 2*goal_point_body[1]/self.lookahead**2
        

            angle = np.clip(angle, -0.4189, 0.4189)
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            if location[0] == "-5.620605915255177":
                print("Found it !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.steering_angle = angle
            drive_msg.drive.speed = self.set_speed(angle)
            #steering_angle_velocity: 0.0
              
        else:
            print("Printing Else statement!!!!!")
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
            drive_msg.drive.steering_angle = 0
            drive_msg.drive.speed = 0
        self.drive_pub.publish(drive_msg)


        


                

        




def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main()