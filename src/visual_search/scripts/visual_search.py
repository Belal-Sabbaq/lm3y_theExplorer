#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VisualSearch:
    def __init__(self):
        rospy.init_node("visual_search_node")

        self.bridge = CvBridge()

        # Load the target image (chair)
        self.target_img = cv2.imread("/home/alaa_ros/limo_ws/src/visual_search/target.jpg", 0)  # grayscale
        if self.target_img is None:
            rospy.logerr(" target.jpg not found! Make sure path is correct.")
            exit()

        # ORB detector
        self.orb = cv2.ORB_create()
        self.kp_t, self.des_t = self.orb.detectAndCompute(self.target_img, None)

        # Subscribe to robot camera
        rospy.Subscriber("/limo/color/image_raw", Image, self.camera_callback)
        rospy.loginfo(" Visual search node started. Searching for target...")

    def camera_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect features in current frame
            kp_f, des_f = self.orb.detectAndCompute(gray_frame, None)
            if des_f is None:
                return

            # Match features
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(self.des_t, des_f)

            # Draw matches (optional for visualization)
            match_img = cv2.drawMatches(self.target_img, self.kp_t, gray_frame, kp_f, matches[:20], None, flags=2)
            cv2.imshow("Matches", match_img)
            cv2.waitKey(1)

            # Decide if target found
            if len(matches) > 25:  # threshold
<<<<<<< HEAD
                rospy.loginfo(" Chair FOUND!")
=======
                rospy.loginfo(" Object FOUND!")
>>>>>>> 1513b60a9a86b82248b8dbb652cd04442d15c0ad
            else:
                rospy.loginfo("Searching...")

        except Exception as e:
            rospy.logerr(e)

if __name__ == "__main__":
    vs = VisualSearch()
    rospy.spin()
