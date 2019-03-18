#!/usr/bin/env python
import random
import time

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np

from robot import bandit_robot

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


goal_pose = PoseStamped()


def scan_callback(msg):
    # print "I received a laser scan from time " + str(msg.header.stamp)
    pass

def scored_callback(msg):
    with open("scores", 'a') as log:
        log.write('\n\n' + str(msg) + "\n" + str(goal_pose))

def map_callback(msg):
    global map_
    print msg.info
    print len(msg.data)
    map_ = np.array(msg.data).reshape((msg.info.height, msg.info.width))
    print(map_.shape)

old = (100, 100)
def move_to_goal(x, y, goal_pub):
    print "I will drive to", x, ",", y, "now!"
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    goal_pose.pose = goal.target_pose.pose
    goal_pose.header.frame_id = 'map'
    goal_pub.publish(goal_pose)

    move_base.send_goal(goal)
    move_base.wait_for_result()




def patrol_fixed_coords():
    route = [
       (0, 0),
       (4.5, -.7),
       (0, -6),
       (9, -5)
    ]

    x, y = route[0]

    while not rospy.is_shutdown():
        old_x, old_y = x, y
        x, y = random.choice(route)
        if (x, y) == (old_x, old_y):
            continue
        move_to_goal(x, y)


def euler_from_quaternions(x,y,z,w):
    q = (x,y,z,w)
    return tf.transformations.euler_from_quaternions(q)


def patrol_samples(goal_pub):
    global old
    threshold = 2
    while not rospy.is_shutdown():
        distance = 0
        xo, yo = old
        while distance < threshold:
            x, y = sample_goal()
            distance = ((xo - x) ** 2 + (yo - y) ** 2) ** (1.0 / 2.0)
        old = (x, y)
        move_to_goal(x, y, goal_pub)


def sample_goal():

    map_ = "euro"

    if map_ == "euro":
        return random.randint(-10, 0), random.randint(-1, 6)

    rand = random.randint(0, 3)
    if rand == 0:
    	return random.uniform(9, 11), random.uniform(-6, -4)
    if rand == 1:
        return random.uniform(3, 6), random.uniform(-5, -1)
    if rand == 2:
        return random.uniform(-1, 1), random.uniform(-1, 1)


if __name__ == '__main__':
    # initialization
    rospy.init_node('behavior')
    goal_pub = rospy.Publisher('goal', PoseStamped, queue_size=1)
    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
    scan_sub = rospy.Subscriber('laser_scan_filtered', LaserScan, scan_callback, queue_size=1)
    hit_sub = rospy.Subscriber('scored_observation', Header, scored_callback, queue_size=1)
    map_ = None
    map_sub = rospy.Subscriber('map', OccupancyGrid, map_callback, queue_size=1)

    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base.wait_for_server()

    rospy.loginfo("Successfully initialized")

    # patrol_fixed_coords()
    patrol_samples(goal_pub)
