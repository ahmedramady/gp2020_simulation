#!/usr/bin/env python
from geometry_msgs.msg import Pose 
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import rospy
import time

rospy.init_node("odometry")

pub = rospy.Publisher('odom', Odometry, queue_size=50)

odom_msg = Odometry()
odom_msg.child_frame_id = "base_link"
odom_msg.header.frame_id = "odom"

def callback(data):
    global pub
    car_pose = data.pose[len(data.pose)-1]
    car_twist = data.twist[len(data.twist)-1]
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.pose.pose = car_pose
    odom_msg.twist.twist = car_twist
    pub.publish(odom_msg)
    
def main():
    sub = rospy.Subscriber('/gazebo/model_states', ModelStates, callback)
    rospy.spin()

if __name__ == "__main__":
    main()