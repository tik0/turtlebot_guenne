#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
from std_srvs.srv import Empty

# global controllers
cmd_vel = None
move_base = None

cmd_twist = Twist()


def scan_callback(msg):
    # print "I received a laser scan from time " + str(msg.header.stamp)
    pass

def scored_callback(msg):
    # print "I scored"
    pass

def pose_callback(msg):
    # print "I got a pose"
    # print(str(msg))
    pass

def clear_cost():
    clear_proxy = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
    try:
        clear_proxy()
    except rospy.ServiceException, e:
        print("Service call failed: %s" %e)

def euclidean_distance(x, y):
    return np.sqrt(x ** 2 + y ** 2)

if __name__ == '__main__':
    # initialization
    rospy.init_node('behavior')
    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
    scan_sub = rospy.Subscriber('laser_scan_filtered', LaserScan, scan_callback, queue_size=1)
    pos_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, pose_callback, queue_size=1)
    hit_sub = rospy.Subscriber('scored_observation', Header, scored_callback, queue_size=1)

    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base.wait_for_server()

    rospy.loginfo("Successfully initialized")

    # run in a circle forever
    # cmd_twist.linear.x = 0.2
    # cmd_twist.angular.z = 0.7

#    r = rospy.Rate(10)
#    while not rospy.is_shutdown():
#	cmd_vel.publish(cmd_twist)
#        r.sleep()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.orientation.w = 1.0
    # run towards a point on the map
    # x_list = [-1, 0, 2, 3, 7, 10 , 3, 7, 2, 5]  # kindergarten
    # y_list = [-1,-6,-2,-4,-7,-4.5,-5,-6,-5,-1]

    x_list = [-8.5,-8,-8,-6,-6 ,-5 ,-4,-3,-2,-2,-5.5, 0]   # europasaal
    y_list = [   0, 2, 3, 4,2.5,5.5, 1, 2, 3, 1,-1.5, 0]
    clear_counter = 0
    distance_to_previous = 6
    
    idx = np.random.choice(range(len(x_list)))
    past_idx = [0, 0]
    distance_to_previous = euclidean_distance(
       				x_list[idx],
       				y_list[idx])
    while not rospy.is_shutdown():
	while idx == past_idx[0] or idx==past_idx[1]:
	    idx = np.random.choice(range(len(x_list)))
	distance_to_previous = euclidean_distance(
            			goal.target_pose.pose.position.x - x_list[idx],
            			goal.target_pose.pose.position.y - y_list[idx])
		
	while distance_to_previous < 2:
            idx = np.random.choice(range(len(x_list)))
	    distance_to_previous = euclidean_distance(
            				goal.target_pose.pose.position.x - x_list[idx],
            				goal.target_pose.pose.position.y - y_list[idx])
	past_idx[0] = past_idx[1]
        past_idx[1] = idx
        goal.target_pose.pose.position.x = x_list[idx]
        goal.target_pose.pose.position.y = y_list[idx]
        move_base.send_goal(goal)
        move_base.wait_for_result()
        rot_direct = np.random.choice([-1,1])
        tnull = time.time()
	currentTime = time.time()
        cmd_twist.linear.x = 0.0
        cmd_twist.angular.z = rot_direct * 0.5 * np.pi
	turn_time = np.random.choice([0, 1, 2, 3])
        while currentTime < tnull + turn_time:
	     cmd_vel.publish(cmd_twist)
             currentTime = time.time()
        
        tnull = time.time()
	currentTime = time.time()
        cmd_twist.linear.x = 0.0
        cmd_twist.angular.z = rot_direct * -0.5 * np.pi
	turn_time = np.random.choice([0, 1, 2])
        while currentTime < tnull + turn_time:
             cmd_vel.publish(cmd_twist)
             currentTime = time.time()

	clear_counter += 1
	if clear_counter > 4:
		clear_cost()
		clear_counter = 0
