#!/usr/bin/env python3
import rospy
import random
from sensor_msgs.msg import LaserScan

STATIC_NOISE = 0.02
DYNAMIC_STD = 0.01

def callback(scan):
    noisy = LaserScan()
    noisy = scan  # copy original scan

    noisy.ranges = [
        max(0.0, r + STATIC_NOISE + random.gauss(0, DYNAMIC_STD)) if r != float('inf') and r == r else r
        for r in scan.ranges
    ]

    pub.publish(noisy)

def main():
    global pub
    rospy.init_node("laser_noise_node")
    pub = rospy.Publisher("/nscan", LaserScan, queue_size=10)
    rospy.Subscriber("/limo/scan", LaserScan, callback)
    rospy.spin()

if __name__ == "__main__":
    main()
