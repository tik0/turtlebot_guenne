#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# global controllers
cmd_vel = None
move_base = None

cmd_twist = Twist()


def scan_callback(msg):
    # print "I received a laser scan from time " + str(msg.header.stamp)
    pass

def scored_callback(msg):
    # print "I scored"
    print(msg)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = 1.9
    goal.target_pose.pose.position.y = 1.3
    orientation = quaternion_from_euler(0, 0, -3.14/2)
    goal.target_pose.pose.orientation.x=orientation[0]
    goal.target_pose.pose.orientation.y=orientation[1]
    goal.target_pose.pose.orientation.z=orientation[2]
    goal.target_pose.pose.orientation.w=orientation[3]
    move_base.send_goal(goal)
    move_base.wait_for_result()


if __name__ == '__main__':
    # initialization
    rospy.init_node('behavior')
    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
    scan_sub = rospy.Subscriber('laser_scan_filtered', LaserScan, scan_callback, queue_size=1)
    hit_sub = rospy.Subscriber('scored_observation', Header, scored_callback, queue_size=1)

    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base.wait_for_server()

    rospy.loginfo("Successfully initialized")

    # run in a circle forever
    cmd_twist.linear.x = 0.2
    cmd_twist.angular.z = 0.7

    r = rospy.Rate(90)
    x_vect = [5.9,  9.92,  1.9,  -1.42  , 7.1,  4.4,  6.03,  6.45,  4.83,  2.86]
    y_vect = [-5.4, -4.76, -5.2, -0.68  , -2.5, -4.4, -2.01, -2.50, -0.83,  -1.58]
    i=0
    while not rospy.is_shutdown():
    #    cmd_vel.publish(cmd_twist)
    #    r.sleep()


    # run towards a point on the map
        i = np.random.randint(len(x_vect))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = x_vect[i]
        goal.target_pose.pose.position.y = y_vect[i]
        orientation = quaternion_from_euler(0, 0, 3.14/2)
        goal.target_pose.pose.orientation.x=orientation[0]
        goal.target_pose.pose.orientation.y=orientation[1]
        goal.target_pose.pose.orientation.z=orientation[2]
        goal.target_pose.pose.orientation.w=orientation[3]
        move_base.send_goal(goal)
        move_base.wait_for_result()
        r.sleep()
