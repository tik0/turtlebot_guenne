#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

cmd_vel = None
move_base = None

cmd_twist = Twist()

if __name__ == '__main__':
    rospy.init_node('behavior')
    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)

    cmd_twist.linear.x = 0.2
    cmd_twist.angular.z = 0.7

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        cmd_vel.publish(cmd_twist)
        r.sleep()
