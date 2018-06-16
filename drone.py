#!/usr/bin/env python
##import required libraries
import roslib
import rospy
import time
import math

##import message types
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
current_position = [4.13, 1.79, 12.47]
prev_position=[4.13, 1.79, 12.47]
waypts = [[4.25, 4.05, 11.0],								#First Waypoint
          [-4.0, 3.25, 11.0],								#Second waypoint
          [-3.8, -4.2, 11.0],								#Third waypoint
          [4.25, -4.2, 11.0],								#Forth waypoint
          [0.0,  0.0, 11.0]]								#Fifth waypoint

o =[0,0,0]
errsum=[0,0,0]
kp=[0.7,0.7,0.7]
kd=[16,16,16]
ki=[0.6,0.6,0.6]
prev_time=time.time()
def getMarkerPose(data):
	global current_position
        global o											
	current_position = [data.poses[0].position.x, data.poses[0].position.y, data.poses[0].position.z]
def direction(pts):
        global prev_time
        global errsum
        global current_position
        global prev_position
        global kp
        global kd
        global ki
        now=time.time()
        print current_position
        for i in range(0,3):
               error=pts[i]-current_position[i]
              
               if(abs(error)>.2):
                       errsum[i] +=error*(now-prev_time)   
                       if errsum[i]>.2:
                               errsum[i]=.2
                       elif errsum[i]<-.2:
                               errsum[i]=-.2
                       d=(pre_error[i]-error)
		       prop=kp[i]*error
		       if prop>.8:
			   prop=.8
		       elif prop<-.8:
			   prop=-.8
                       o[i]=kp[i]*error+ki[i]*errsum[i]-kd[i]*d
                       if o[i]>.9:
	                       o[i]=.4
                       elif o[i]<-.9:
                               o[i]=-.4
                       
               else:
                      o[i]=0
        prev_time=now
	prev_position=current_position
        print(o)
        return o
if __name__=="__main__":
        pub_empty_takeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
        rospy.init_node('issue_motion')  						
	rospy.Subscriber('/whycon/poses',PoseArray,getMarkerPose)
        move_drone=rospy.Publisher('/cmd_vel', Twist , queue_size=1)
        pub_empty_land=rospy.Publisher('/ardrone/land' ,Empty ,queue_size=1)
        pub_empty_takeoff.publish(Empty())
	twist=Twist()
        for pts in waypts:							 
               while 1:					
                       pub_empty_takeoff.publish(Empty())
                       k=direction(pts)
                       if k[0]==0 and k[1]==0 and k[2]==0:
                               break
                       twist.linear.x=-k[1]
                       twist.linear.y=-k[0]
                       twist.linear.z=-k[2]
		       twist.angular.x=0.0
                       twist.angular.y=0.0
                       twist.angular.z=0.0
                       move_drone.publish(twist)
        pub_empty_land.publish(Empty()) 
