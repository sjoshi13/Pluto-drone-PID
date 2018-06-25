#!/usr/bin/env python
##import required libraries
import roslib
import rospy
import time
import math
import numpy as points
##import message types
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
current_position = [0,0,12.47]
prev_position = [0,0,12.47]
n=points.linspace(0,2*math.pi,10)
waypts = [0,0,12.3]
point=[]
o =[0.0,0.0,0.0]
pre_error=[0.0,0.0,0.0]
errsum=[0.0,0.0,0.0]
kp=[.5,.5,.5]
kd=[16,16,16]
ki=[.5,.5,.5]
prev_time=time.time()
def get_coor(i,radius,cx,cy):
        global waypts
        waypts[0]=radius*((math.cos(n[i]))+cx)
        waypts[1]=radius*((math.sin(n[i]))+cy)
        print "next coordinate" + str(waypts)
        point.append(waypts)
        return waypts       
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
        global pre_error
        now=time.time()
        print "currentPosition"+str(current_position)
        print "moving to point "+str(pts)
        for i in range(0,3):
               print "i  :"+str(i)
               error=pts[i]-current_position[i]
               print "error"+str(error)
               if(abs(error)>.2):
                       print "now - prev"+str(now-prev_time)
                       errsum[i] +=error*(now-prev_time)
                       print "errsum for i"+str(i)+"is"+str(errsum[i])  
                       print errsum[i]
                       if errsum[i]>.2:
                               errsum[i]=.2
                       elif errsum[i]<-0.2:
                               errsum[i]=-.2
                       d=(pre_error[i]-error)
                       print "d"+str(d)
                       prop=kp[i]*error
                       if prop>.6:
                          prop=0.6
                       elif prop<-0.4:
                           prop=-.4
                       o[i]=prop+ki[i]*errsum[i]-kd[i]*d
                       print "o[i] for i = "+str(i)+"  "+str(o[i])    
                       if o[i]>.9:
	                       o[i]=.9
                               print "o for i"+str(i)+" "+str(o)
                       elif o[i]<-.9:
                               o[i]=-.9
               else:
                       o[i]=0.0
        prev_time=now
        prev_position=current_position
        print "prepos  :"+str(prev_position)
        print(o)
        return o
if __name__=="__main__":
        pub_empty_takeoff = rospy.Publisher('/takeoff', Empty, queue_size=1)
        rospy.init_node('issue_motion')  						
	rospy.Subscriber('/whycon/poses',PoseArray,getMarkerPose)
        move_drone=rospy.Publisher('/cmd_vel', Twist , queue_size=1)
        pub_empty_land=rospy.Publisher('/land',Empty ,queue_size=1)
        pub_empty_takeoff.publish(Empty())
        twist=Twist()
        for i in range (0,10):							 
               while 1:			
                       pub_empty_takeoff.publish(Empty())	
                       pts=get_coor(i,3,0,0)#i, radius,centerx,centery
                       print(pts)
                       k=direction(pts)
                       if k[0]==0 and k[1]==0 and k[2]==0:
                               rospy.sleep(.09)
                               pub_empty_land.publish(Empty()) 
                               break
                       twist.linear.x=-k[1]
                       twist.linear.y=-k[0]
                       twist.linear.z=-k[2]
                       twist.angular.x=0.0
                       twist.angular.y=0.0
                       twist.angular.z=0.0
                       move_drone.publish(twist)
        while 1:				
               pts=get_coor(9,3,0,0)#landing point,radius,centerx,centery
               print(pts)
               k=direction(pts)
               if k[0]==0 and k[1]==0 and k[2]==0:
                       pub_empty_land.publish(Empty()) 
                       break
               twist.linear.x=-k[1]
               twist.linear.y=-k[0]
               twist.linear.z=-k[2]
               twist.angular.x=0.0
               twist.angular.y=0.0
               twist.angular.z=0.0
               move_drone.publish(twist)
        pub_empty_land.publish(Empty()) 
