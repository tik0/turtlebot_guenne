#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from apriltags2_ros.msg import AprilTagDetectionArray
from math import pi, atan2, log, copysign
import tf
from threading import Lock
from actionlib_msgs.msg import GoalStatus

# global controllers
cmd_vel = None
move_base = None

cmd_twist = Twist()

movement_lock = Lock()

tag_angle_threshold = 0.05
tag_dist_threshold = 0.4
tag_lost_angle = 0.1
is_following = False
tag_vels=[]
last_tag_seen = None
last_goal_succeeded = None
turning_after_goal = 7.5

def clear_cost():
	clear_proxy = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
	
	try:
		clear_proxy()
	except:
		print('failed clearing')


def scan_callback(msg):
	# print "I received a laser scan from time " + str(msg.header.stamp)
	pass

def scored_callback(msg):
	# print "I scored"
	pass

def follow(angle,z):
	cmd_twist = Twist()
	#print(angle,z)
	if angle > tag_angle_threshold:
	   cmd_twist.angular.z = -5*angle/pi
	elif angle < -tag_angle_threshold:
	   cmd_twist.angular.z = -5*angle/pi
	   
	if z > tag_dist_threshold:
		cmd_twist.linear.x=min(0.5,0.8*(z-tag_dist_threshold))
	else:
		cmd_twist.angular.z *= 1.5
		
		
	return cmd_twist

def tag_callback(msg):
	#print(msg)
	global tag_vels
	global movement_lock
	global is_following
	global last_tag_seen
	if(len(msg.detections) == 0):
		if not len(tag_vels) == 0:
			move_base.cancel_all_goals()
			cmd_twist=follow(tag_vels[0],tag_vels[1])
			cmd_vel.publish(cmd_twist)
			tag_vels[0]-=copysign(0.02,tag_vels[0])
			tag_vels[1]-=0.07
			
			if tag_vels[0]<tag_angle_threshold and tag_vels[1]<0:
				tag_vels=[]
				is_following = False
				print('not following')
				
		# TODO: What to do now?
		return
		
	last_tag_seen = msg.header.stamp.secs
	is_following = True
	print('is following')
	#with movement_lock:
	move_base.cancel_all_goals()
	#move_base.cancel_goal()
	pos = msg.detections[0].pose.pose.pose.position
	angle = atan2(pos.x,pos.z)
	#orientation_quat = [orientation.x, orientation.y, orientation.z, orientation.w]
	#euler=tf.transformations.euler_from_quaternion(orientation_quat)
	#print(euler)
	#new_orientation=tf.transformations.quaternion_from_euler(0.0,0.0,euler[2])
	#print(new_orientation)
	cmd_twist=follow(angle,pos.z)
	cmd_vel.publish(cmd_twist)
	#if angle > tag_lost_angle or angle < -tag_lost_angle:
	tag_vels=[]
	tag_vels.append(angle)
	tag_vels.append(pos.z)
#	goal = MoveBaseGoal()
#	
#	goal.target_pose.header.frame_id = 'map'
#	goal.target_pose.pose.orientation.x=new_orientation[0]
#	goal.target_pose.pose.orientation.y=new_orientation[1]
#	goal.target_pose.pose.orientation.z=new_orientation[2]
#	goal.target_pose.pose.orientation.w=new_orientation[3]
#	move_base.send_goal(goal)
#	move_base.wait_for_result()


def turn_ccw():
	cmd_twist = Twist()
	#cmd_twist.linear.x = 0.2
	cmd_twist.angular.z = 0.7
	step = 0
	
	r = rospy.Rate(10)
	while step < 50:
		cmd_vel.publish(cmd_twist)
		#print(step)
		step += 1
		
		if step == 25:
			rospy.sleep(1.5)
		
		r.sleep()
		

def goal_done(state, result):
	global last_goal_succeeded
	rospy.loginfo('Done!')
	
	if state == GoalStatus.SUCCEEDED:
		last_goal_succeeded = rospy.get_time()
		rospy.sleep(1.)
		turn_ccw()	
		rospy.loginfo('...with success!!!')
		
	#else:
	#	print('with state', state)	
	
	
def goal_feedback(feedback):
	pass
	#print(feedback)			

if __name__ == '__main__':
	# initialization
	rospy.init_node('behavior')
	tag_vels=[]
	
	cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
	scan_sub = rospy.Subscriber('laser_scan_filtered', LaserScan, scan_callback, queue_size=1)
	hit_sub = rospy.Subscriber('scored_observation', Header, scored_callback, queue_size=1)
	tag_sub = rospy.Subscriber('tag_detections', AprilTagDetectionArray, tag_callback, queue_size=1)

	move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	move_base.wait_for_server()

	rospy.loginfo("Successfully initialized")

	# run in a circle forever
	#cmd_twist = Twist()
	#cmd_twist.linear.x = 0.2
	#cmd_twist.angular.z = 0.7

	r = rospy.Rate(1)
		#cmd_vel.publish(cmd_twist)

	path=[(0,-5.75, 0.22), # Stairs
		  (5.5,-5.0, 2.16), #Left of small corridor
		  (6.7,-4.1, 2.7), #Look into Mauseloch
		  (9.5,-4.8, 2.7), #Drive into Mauseloch
		  (4.5,-0.7, 0.494), #Middle room
		  (2.5, -1.5, 6.0), #Look into our room
		  (-0.53, -0.18, 5.8)] #Drive into our room
	path = []	  
	# run towards a point on the map
	#angle=0.0
	while not rospy.is_shutdown():
		for point in path:
			while last_tag_seen > rospy.get_time() - 2 \
				or last_goal_succeeded > rospy.get_time() - (turning_after_goal+1):
				r.sleep()
			
			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = 'map'
			goal.target_pose.pose.position.x = point[0]
			goal.target_pose.pose.position.y = point[1]
			orientation=tf.transformations.quaternion_from_euler(0.0,0.0,point[2] - pi)
			
			goal.target_pose.pose.orientation.x=orientation[0]
			goal.target_pose.pose.orientation.y=orientation[1]
			goal.target_pose.pose.orientation.z=orientation[2]
			goal.target_pose.pose.orientation.w=orientation[3]
			rospy.loginfo('sending goal...')
			move_base.send_goal(goal, done_cb=goal_done, feedback_cb=goal_feedback)
			if not move_base.wait_for_result():
				rospy.loginfo('failed moving')
				clear_cost()
				rospy.loginfo(move_base.get_result())
			rospy.loginfo('...got result.')
			rospy.sleep(2.)
		r.sleep()
