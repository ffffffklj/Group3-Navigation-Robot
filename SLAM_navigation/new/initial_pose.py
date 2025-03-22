#!/usr/bin/env python3
 
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
 
rospy.init_node('init_pos')
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 10)

rospy.sleep(3)
checkpoint = PoseWithCovarianceStamped()

# Use $ rostopic echo /odom -n1 to obtain poses (both position and orientation)

checkpoint.pose.pose.position.x = -2.9999990707712256
checkpoint.pose.pose.position.y = 1.0000168952874815
checkpoint.pose.pose.position.z = -0.0010073991464728017
 
[x,y,z,w]=quaternion_from_euler(0.0,0.0,0.0)
checkpoint.pose.pose.orientation.x = -1.7451239793023515e-06
checkpoint.pose.pose.orientation.y = 0.0015896503142578304
checkpoint.pose.pose.orientation.z = 5.246788792915036e-05
checkpoint.pose.pose.orientation.w = 0.999998735127177
 
print(checkpoint)
pub.publish(checkpoint)
	
