#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

#Time function to return current time
def time():
    return rospy.get_rostime().secs + (rospy.get_rostime().nsecs/1e9)
    
#Stop function to publish Twist msg with all velocity components as 0
def stop( pub):
    t=Twist()
    t.linear.x=0
    t.linear.y=0
    t.linear.z=0
    t.angular.x=0
    t.angular.y=0
    t.angular.z=0
    pub.publish(t)


def move():
    pub= rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('movement_robot', anonymous=True)
    rospy.sleep(1) #allows time for publisher to connect
    tempo= time()
    while not rospy.is_shutdown():
        current_time= time()
       
       #Move forward for 3 seconds at 1m/s 
        if current_time- tempo<=3:
            t=Twist()
            t.linear.x=1
            t.angular.z=0
            pub.publish(t)
            
    #stop for 1sec        
        elif current_time- tempo >3 and current_time- tempo <=4:
            stop(pub)
            rospy.sleep(1)
            
    #Rotate anticlockwise for 2 seconds at 1 rad/s    
        elif current_time- tempo > 4 and current_time - tempo <=6:
            t=Twist()
            t.linear.x=0
            t.angular.z=1
            pub.publish(t)
    
    #Stop for 1 sec
        elif current_time- tempo >6 and  current_time- tempo <=7:
            stop(pub)
            rospy.sleep(1)
            
    #Move backward for 3 seconds at -0.5 m/s   
        elif current_time- tempo > 7 and current_time - tempo <=10:
            t=Twist()
            t.linear.x=-0.5
            t.angular.z=0
            pub.publish(t)
            
  #Stop for 1 sec
        elif current_time- tempo >10 and  current_time- tempo <=11:
            stop(pub)
            rospy.sleep(1)
            
        else:
            break
            
if __name__ == '__main__':
   try:
       move()
   except rospy.ROSInterruptException:
       pass

