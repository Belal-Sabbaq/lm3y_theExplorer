#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

def callback(msg):
    try:
        # Convert ROS image to OpenCV image
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("Limo Camera", img)

        key = cv2.waitKey(1)
        if key == ord('s'):  # Press 's' to save frame
            cv2.imwrite("/home/alaa_ros/limo_ws/src/visual_search/target.jpg", img)
            rospy.loginfo("target.jpg saved!")

    except Exception as e:
        rospy.logerr(e)

rospy.init_node("image_viewer")
rospy.Subscriber("/limo/color/image_raw", Image, callback)
rospy.loginfo("Viewing /limo/color/image_raw ... Press 's' to save target image")
rospy.spin()

