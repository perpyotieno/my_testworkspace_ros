#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from math import floor

# get the laser messages
def callback_laser(msg):
   # 0 graus: behind the robot
   # 90 graus: right side of the robot
   # 180 graus: in front of the robo
   # 270 graus: left side of the robot
   laser_raw = msg.ranges #retrieves raw range data from the msg-lists of distances measured by the laser from various angles
   laser_float = [float(r) for r in laser_raw] #converts each value to a float
   print("Position 0 graus = " + str(laser_float[0]))
   print("Position 45 graus = "  + str(laser_float[int(45*2)-1]))
   print("Position 90 graus = "  + str(laser_float[int(90*2)-1]))
   print("Position 135 graus = "  + str(laser_float[int(135*2)-1]))
   print("Position 180 graus = "  + str(laser_float[int(180*2)-1]))
   print("Position 225 graus = "  + str(laser_float[int(225*2)-1])) 
   print("Position 270 graus = "  + str(laser_float[int(270*2)-1])) 
   print("Position 305 graus = "  + str(laser_float[int(305*2)-1]))
   
if __name__ == '__main__':
   rospy.init_node("obstacle_check_node")
   rospy.Subscriber("/rslidar_points", LaserScan, callback_laser)
   print("Entrei")
   rospy.spin() # this will block untill you hit Ctrl+C