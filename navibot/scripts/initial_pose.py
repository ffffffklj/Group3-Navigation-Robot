#!/usr/bin/env python3
 
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
 
rospy.init_node('init_pos')
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 10)

rospy.sleep(3)
checkpoint = PoseWithCovarianceStamped()

# Use $ rostopic echo /odom -n1 to obtain poses (both position and orientation)

checkpoint.pose.pose.position.x = -1.999970
checkpoint.pose.pose.position.y = -0.499905
checkpoint.pose.pose.position.z = -0.001007
 
[x,y,z,w]=quaternion_from_euler(0.0,0.0,0.0)
checkpoint.pose.pose.orientation.x = -2.5422331743973297e-05
checkpoint.pose.pose.orientation.y = 0.0015896503142578304
checkpoint.pose.pose.orientation.z = 0.015765793325115327
checkpoint.pose.pose.orientation.w = 0.999998735127177
 
print(checkpoint)
pub.publish(checkpoint)
	
