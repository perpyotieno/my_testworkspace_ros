#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import atan2, sqrt

class ObstacleAvoidanceNode:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node')
        self.cmd_vel_pub =rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan,self.laser_callback)
        self.speed = Twist()
        self.rate =rospy.Rate(10)

    def laser_callback(self, data):
        #process laser scan data
        front_distance = min(data.ranges[0:30] + data.ranges[-30:])
        #Front distance check
        if front_distance < 0.5: #if an obstacle is closer than 0.5 meters
            self.speed.linear.x = 0.0 # stop moving forward
            self.speed.angular.z = 0.5 # turn to avoid bstacle
        else:
            self.speed.linear.x =0.5 # Move forward
            self.speed.angular.z = 0.0 # No rotation
            
        self.cmd_vel_publish (self.speed)
        
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            
if __name__ == '__main__':
    try:
        node = ObstacleAvoidanceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass