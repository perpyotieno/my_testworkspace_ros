#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

# Initialize global variables
pub = None

def laser_callback(msg):
    global pub
    
    #To check from the front of the robot
    front_index = int(len(msg.ranges)/2)
    
    #centre index at 0 degrees
    distance_to_pillar= msg.ranges[front_index]
    
    if distance_to_pillar < msg.range_max and distance_to_pillar>msg.range_min:
        
        angle= msg.angle_min + front_index * msg.angle_increment
        
        x_position = distance_to_pillar * math.cos(angle)
        y_position = distance_to_pillar * math.sin(angle)
        
        rospy.loginfo(f"Pillar detected at(x: {x_position:.2f}, y: {y_position:.2f})")
        
    def stop_robot():
        global pub
        twist = Twist()
        pub.publish(twist)
        
    if __name__== '__name__':
        try:
            rospy.init_node('pillar_detector')
            
            pub=rospy.Publisher('/cmd_vel',Twist, queue_size=10)
            rospy.Subscriber('/rslidar_points', LaserScan, laser_callback)
            
            rospy.spin()
        except rospy.ROSInterruptException:
            pass   