#!/usr/bin/env python
from lib2to3.pgen2 import driver
import rospy
from std_msgs.msg import Float32 
import numpy as np
import atexit
import rospkg 
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from sensor_msgs.msg import LaserScan, LaserEcho
class SafetyAggregator:
    def __init__(self):

        self.safe_count = 0
        self.unsafe_count = 0
        self.start_time = None
        self.total_count = 0 
        self.algorithm_name = "Follow the Gap algorithm"
        data = LaserScan()
        self.velocity = data.angle_min
        self.experiment_number = 0 
        # use the rospack object to get paths
        rospack = rospkg.RosPack()
        odom_topic = '/scan'
        package_path=rospack.get_path('f1tenth_simulator')+"/logs/collection1235.csv"
        self.file = open(package_path, 'a')
        self.reachability_result=rospy.Subscriber(odom_topic,LaserScan,self.callback)
      

    def callback(self,results):
        check = results
        checkup = LaserScan()
        if(self.total_count==0):
            self.start_time = rospy.Time.now()
        if(checkup.range_min >= 0.05):
            self.start_time = rospy.Time.now()
            self.unsafe_count +=1
        else:
            self.safe_count+=1
        self.total_count+=1

  
    def shutdown(self):
        safety_percentage = self.safe_count/float(self.total_count)
        self.file.write("{}, {}, {}, {}\n".format(self.algorithm_name,self.unsafe_count,self.safe_count,safety_percentage))
        self.file.close() 

        
        
if __name__=='__main__':
    rospy.init_node("safety_aggregator",anonymous=True)
    #get the arguments passed from the launch file
    r = rospy.Rate(120)

    res_node = SafetyAggregator()
    atexit.register(res_node.shutdown)
    while not rospy.is_shutdown():
        r.sleep()
    

    