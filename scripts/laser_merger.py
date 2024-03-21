#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan

import ira_laser_tools

import math

ira_laser_tools.