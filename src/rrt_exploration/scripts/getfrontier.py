#!/usr/bin/env python3

# --------Include modules---------------
from copy import copy
import rospy
from nav_msgs.msg import OccupancyGrid

import numpy as np
import cv2

# -----------------------------------------------------


def getfrontier(mapData: OccupancyGrid):
    data = mapData.data
    w = mapData.info.width
    h = mapData.info.height
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y

    # Single-channel image is enough
    img = np.zeros((h, w), np.uint8)

    # Build map image: 0 = occupied, 255 = free, 205 = unknown
    for i in range(h):
        for j in range(w):
            val = data[i * w + j]
            if val == 100:
                img[i, j] = 0
            elif val == 0:
                img[i, j] = 255
            elif val == -1:
                img[i, j] = 205

    # Threshold occupied (0..1) area
    o = cv2.inRange(img, 0, 1)

    # Edges of free/unknown
    edges = cv2.Canny(img, 0, 255)

    # findContours signature differs between OpenCV versions, so be robust
    def find_contours(src):
        try:
            # OpenCV 3/early 4 style
            _, contours, hierarchy = cv2.findContours(
                src, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )
        except ValueError:
            # OpenCV 4+ style
            contours, hierarchy = cv2.findContours(
                src, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )
        return contours, hierarchy

    # Dilate occupied regions
    contours, hierarchy = find_contours(o)
    cv2.drawContours(o, contours, -1, (255, 255, 255), 5)

    o = cv2.bitwise_not(o)
    res = cv2.bitwise_and(o, edges)

    # ------------------------------ frontier extraction -------------------------
    frontier = copy(res)

    contours, hierarchy = find_contours(frontier)
    cv2.drawContours(frontier, contours, -1, (255, 255, 255), 2)

    contours, hierarchy = find_contours(frontier)
    all_pts = []

    if len(contours) > 0:
        for cnt in contours:
            M = cv2.moments(cnt)
            if M["m00"] == 0:  # avoid division by zero
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            xr = cx * resolution + Xstartx
            yr = cy * resolution + Xstarty

            pt = [np.array([xr, yr])]
            if len(all_pts) > 0:
                all_pts = np.vstack([all_pts, pt])
            else:
                all_pts = pt

    return all_pts
