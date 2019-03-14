#!/usr/bin/env python

import rospy
import message_filters

from rpc_game_client.srv import PlayerScore

from sensor_msgs.msg import CompressedImage, CameraInfo
from apriltags2_ros.msg import AprilTagDetectionArray

class Scoring:
    def __init__(self):
        self.last_score_time = rospy.get_rostime() - rospy.Duration(20)
        rospy.loginfo("waiting for score service")
        rospy.wait_for_service('rpc_score')
        rospy.loginfo("Found score service")

        image_sub = message_filters.Subscriber('/camera/rgb/image_color/compressed', CompressedImage)
        info_sub = message_filters.Subscriber('/camera/rgb/camera_info', CameraInfo)
        apriltag_sub = message_filters.Subscriber("/tag_detections", AprilTagDetectionArray)
        ts = message_filters.TimeSynchronizer([image_sub, info_sub, apriltag_sub], 10)

        ts.registerCallback(self.callback)
    

    def send_score_image(self, image, camera_info):
        try:
            player_score = rospy.ServiceProxy('rpc_score', PlayerScore)
            resp = player_score(image, camera_info)
            self.last_score_time = rospy.get_rostime()

            rospy.loginfo("scored: " + str(resp))
        except rospy.ServiceException, e:
            rospy.logwarn("Failed to score: %s"%e)

    def callback(self, image, camera_info, apriltag):
        if (rospy.get_rostime()-self.last_score_time) >= rospy.Duration(10) and len(apriltag.detections) > 0:
            self.send_score_image(image, camera_info)
 
if __name__ == "__main__":
    rospy.init_node("score_observed_robots")

    scoring = Scoring()

    rospy.spin()
