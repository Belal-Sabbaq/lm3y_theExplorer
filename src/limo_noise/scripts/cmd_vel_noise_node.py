#!/usr/bin/env python3
import rospy
import random
from geometry_msgs.msg import Twist

STATIC_NOISE = 0.02
DYNAMIC_STD = 0.01

def callback(msg):
    noisy = Twist()
    noisy.linear.x  = msg.linear.x  + STATIC_NOISE + random.gauss(0, DYNAMIC_STD)
    noisy.angular.z = msg.angular.z + STATIC_NOISE + random.gauss(0, DYNAMIC_STD)
    pub.publish(noisy)

def main():
    global pub
    rospy.init_node("cmd_vel_noise_node")
    pub = rospy.Publisher("/ncmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == "__main__":
    main()
