#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import math

class PillarDetector:
    def __init__(self):
        #initialize the ROS node
        rospy.init_node('pillar_detector', anonymous=True)
        
        #publisher for velocity
        self.pub_cmd_vel=rospy.Publisher('/cmd_vel',Twist, queue_size=10)
            
        #publisher for visualization markers
        self.pub_marker=rospy.Publisher('/visualization_marker', Marker,queue_size=10)
        
        #subscriber for laser scan data
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        #set proportional gain and threshold distances for stopping
        self.p_gain = 1.0
        self.threshold_distance = 1.0
    
    def laser_callback(self, msg):
        #To check from the front of the robot
        front_index = int(len(msg.ranges)/2)
        
        #centre index at 0 degrees
        distance_to_pillar= msg.ranges[front_index]
        
        if msg.range_min < distance_to_pillar < msg.range_max:
            angle = msg.angle_min + front_index * msg.angle_increment
            x_position = distance_to_pillar * math.cos(angle)
            y_position = distance_to_pillar * math.sin(angle)
            rospy.loginfo(f"Pillar detected at(x: {x_position:.2f}, y: {y_position:.2f})")
            
            #Create and configure the marker
            marker= Marker()
            marker.header.frame_id= "base_link"
            marker.header.stamp= rospy.Time.now()
            marker.ns= "pillar_marker"
            #give a uniqe id to the marker
            marker.id= 0
            #choose the shape of the marker
            marker.type= Marker.SPHERE
            marker.action = Marker.ADD
            #Set the positions of the marker
            marker.pose.position.x= x_position
            marker.pose.position.y= y_position
            marker.pose.position.z=0
            #to avoid rotation
            marker.pose.orientation.w=1.0
        
            #set colour
            marker.color.r=1.0
            marker.color.g=0.0
            marker.color.b=0.0
            marker.color.a=1.0
        
            #set scale of the marker
            marker.scale.x=0.1
            marker.scale.y= 0.1
            marker.scale.z=0.1
        
            #publish marker
            self.pub_marker.publish(marker)
            
            #now the logic for moving towards pillar
            cmd= Twist()
            
            if distance_to_pillar< self.threshold_distance:
                cmd.linear.x = self.p_gain * (self.threshold_distance - distance_to_pillar)
                self.pub_cmd_vel.publish(cmd)
        
        
def main ():
    detector= PillarDetector()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


        
if __name__== '__main__':
    main()