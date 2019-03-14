#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# global controllers
cmd_vel = None
move_base = None

cmd_twist = Twist()


def scan_callback(msg):
    print "I received a laser scan from time " + str(msg.header.stamp)
    pass

if __name__ == '__main__':
    # initialization
    rospy.init_node('behavior')
    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
    scan_sub = rospy.Subscriber('laser_scan_filtered', LaserScan, scan_callback, queue_size=1)

    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base.wait_for_server()

    rospy.loginfo("Successfully initialized")

    # run in a circle forever
    cmd_twist.linear.x = 0.2
    cmd_twist.angular.z = 0.7

    #r = rospy.Rate(10)
    #while not rospy.is_shutdown():
    #    cmd_vel.publish(cmd_twist)
    #    r.sleep()


    # run towards a point on the map
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = 0.0
    goal.target_pose.pose.position.y = 0.0
    goal.target_pose.pose.orientation.w = 1.0
    move_base.send_goal(goal)
    move_base.wait_for_result()

