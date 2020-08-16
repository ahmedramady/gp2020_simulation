#!/usr/bin/env python
from geometry_msgs.msg import Pose 
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Quaternion 
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Polygon

import math
import rospy
import time
import tf

rospy.init_node("odometry")

pub = rospy.Publisher('odom', Odometry, queue_size=50)
pub2 = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
br = tf.TransformBroadcaster()
v3 = Vector3Stamped()

odom_msg = Odometry()
odom_msg.child_frame_id = "base_link"
odom_msg.header.frame_id = "odom"

odom_trans = TransformStamped()
odom_quat = Quaternion()
    
def callback(data):
    pose_stamped_msg = PoseWithCovarianceStamped()
    current_time = rospy.Time.now()
    global pub, pub2, br, v3, odom_quat, odom_trans
    car_pose = data.pose[len(data.pose)-1]
    car_twist = data.twist[len(data.twist)-1]
    v3.vector.x = car_pose.position.x
    v3.vector.y = car_pose.position.y
    v3.vector.z = car_pose.position.z
    roll = car_twist.angular.x
    pitch = car_twist.angular.y
    yaw = car_twist.angular.z
    odom_quat.x = car_pose.orientation.x
    odom_quat.y = car_pose.orientation.y
    odom_quat.z = car_pose.orientation.z
    odom_quat.w = car_pose.orientation.w
#tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    #print('x is {}'.format(v3.vector.x))
    #print('y is {}'.format(v3.vector.y))
    #print('z is {}'.format(v3.vector.z))

    odom_trans.header.stamp = current_time
    odom_trans.header.frame_id = "odom"
    odom_trans.child_frame_id = "base_link"
    odom_trans.transform.translation.x = car_pose.position.x - 10.0
    odom_trans.transform.translation.y = car_pose.position.y + 14.2
    odom_trans.transform.translation.z = 0.0
    odom_trans.transform.rotation = odom_quat
    br.sendTransform((odom_trans.transform.translation.x, odom_trans.transform.translation.y, odom_trans.transform.translation.z),
		      (odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w),
		      current_time,
		      "base_link", "odom")

    odom_msg.header.stamp = current_time
    odom_msg.pose.pose.position.x = car_pose.position.x - 10.0
    odom_msg.pose.pose.position.y = car_pose.position.y + 14.2
    odom_msg.pose.pose.position.z = car_pose.position.z
    odom_msg.pose.pose.orientation = odom_quat
    
    odom_msg.child_frame_id = "base_link"
    odom_msg.twist.twist.linear.x = car_twist.linear.x 
    odom_msg.twist.twist.linear.y = car_twist.linear.y
    odom_msg.twist.twist.linear.z = car_twist.linear.z
    odom_msg.twist.twist.angular.x = car_twist.angular.x
    odom_msg.twist.twist.angular.y = car_twist.angular.y
    odom_msg.twist.twist.angular.z = car_twist.angular.z

    pub.publish(odom_msg)
    
    pose_stamped_msg.header.stamp = current_time
    pose_stamped_msg.header.frame_id = "odom"

    pose_stamped_msg.pose.pose.position.x = odom_msg.pose.pose.position.x
    pose_stamped_msg.pose.pose.position.y = odom_msg.pose.pose.position.y
    pose_stamped_msg.pose.pose.position.z = 0.5

    pose_stamped_msg.pose.pose.orientation.x = odom_quat.x
    pose_stamped_msg.pose.pose.orientation.y = odom_quat.y
    pose_stamped_msg.pose.pose.orientation.z = odom_quat.z
    pose_stamped_msg.pose.pose.orientation.w = odom_quat.w
    

    pub2.publish(pose_stamped_msg)
    
def main():
    sub = rospy.Subscriber('/gazebo/model_states', ModelStates, callback)
    rospy.spin()

if __name__ == "__main__":
    main()
