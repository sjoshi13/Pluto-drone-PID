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
kp=[10.0,10.0,10.0]
kd=[.16,.16,.16]
ki=[16.0,16.0,16.0]
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
               print errsum
               if(error>.2 or error<-.2):
                       print "now - prev"
                       print (now-prev_time)
                       errsum[i] +=error*(now-prev_time)   
                       if errsum[i]>.2:
                               errsum[i]=.2
                       elif errsum[i]<-.2:
                               errsum[i]=-.2
                       d=(error)/((now-prev_time))
                       o[i]=kp[i]*error+ki[i]*errsum[i]-kd[i]*d
                       if o[i]>.2:
	                       o[i]=.1
                       elif o[i]<-.2:
                               o[i]=-.1
                       prev_position[i]=current_position[i]
               else:
                      o[i]=0
        prev_time=now
        print(o)
        return o
if __name__=="__main__":
        pub_empty_takeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
        rospy.init_node('issue_motion')  						
	rospy.Subscriber('/whycon/poses',PoseArray,getMarkerPose)
        move_drone=rospy.Publisher('/cmd_vel', Twist , queue_size=1)
        pub_empty_land=rospy.Publisher('/ardrone/land' ,Empty ,queue_size=1)
        pub_empty_takeoff.publish(Empty())
        for pts in waypts:							 
               while 1:	
                       print "here"	
                       twist=Twist()					
                       pub_empty_takeoff.publish(Empty())
                       k=direction(pts)
                       if k[0]==0 and k[1]==0 and k[2]==0:
                               break
                       twist.linear.x=.01*k[0]
                       twist.linear.y=.01*k[1]
                       twist.linear.z=.01*k[2]
                       move_drone.publish(twist)
        pub_empty_land.publish(Empty()) 
