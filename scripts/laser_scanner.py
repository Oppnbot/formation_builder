#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

from __future__ import annotations

import sensor_msgs.point_cloud2 as pc2
import tf
import rospy
import math
import numpy as np
import ros_numpy
from sensor_msgs.msg import PointCloud2, LaserScan, PointField
from geometry_msgs.msg import TransformStamped
from laser_geometry.laser_geometry import LaserProjection
from visualization_msgs.msg import Marker, MarkerArray
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, Pose
from tf.transformations import euler_from_quaternion

class LaserScanner:
    def __init__(self, robot_id: int) -> None:
        self.initialized : bool = False
        self.id : int = robot_id
        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform("map", f"mir{robot_id}/front_laser_link", rospy.Time(0), rospy.Duration(3))
        self.tf_listener.waitForTransform("map", f"mir{robot_id}/back_laser_link", rospy.Time(0), rospy.Duration(3))
        self.scan_left_subscriber : rospy.Subscriber = rospy.Subscriber(f"/mir{robot_id}/f_scan", LaserScan, self.scanner_callback, 'front')
        self.scan_right_subscriber : rospy.Subscriber = rospy.Subscriber(f"/mir{robot_id}/b_scan", LaserScan, self.scanner_callback, 'back')
        self.transform = TransformStamped()
        self.laser_projection = LaserProjection()

        self.front_laser_points : list[list[float]] = []
        self.back_laser_points : list[list[float]] = []
        self.initialized = True
        return None
    

    def get_laser_points(self) -> list[list[float]]:
        return self.front_laser_points + self.back_laser_points
        return self.front_laser_points.extend(self.back_laser_points)
        return np.vstack((self.front_laser_points, self.back_laser_points))

    
    def scanner_callback(self, scan: LaserScan, side : str) -> None:

        cloud_out : PointCloud2 = self.laser_projection.projectLaser(scan)
        if side == "front":
            (trans, rot) = self.tf_listener.lookupTransform(f"mir{self.id}/base_link", f"mir{self.id}/front_laser_link", rospy.Time(0))
        elif side == "back":
            (trans, rot) = self.tf_listener.lookupTransform(f"mir{self.id}/base_link", f"mir{self.id}/back_laser_link", rospy.Time(0))
        else:
            rospy.logwarn(f"[SCANNER {self.id}]: Unknown Laser Scanner Side: {side}")
            return None
        self.transform.transform.translation.x = trans[0]
        self.transform.transform.translation.y = trans[1]
        self.transform.transform.rotation.x = rot[0]
        self.transform.transform.rotation.y = rot[1]
        self.transform.transform.rotation.z = rot[2]
        self.transform.transform.rotation.w = rot[3]
        cloud_transformed : PointCloud2 = do_transform_cloud(cloud_out, self.transform)


        data : np.ndarray = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(cloud_transformed, remove_nans=True)

        if side == "front":
            #rospy.logwarn("owowowow")
            self.front_laser_points = data.tolist()
        else:
            #rospy.logwarn("UWUWUWUW")
            self.back_laser_points = data.tolist()
        return None


        is_colliding: bool = False
        for point in data:
            if abs(point[1]) < 0.1 and 0 < point[0] < 1:
                is_colliding = True
        if is_colliding:
            rospy.logerr("COLLISION INCOMING!!!")
        else:
            rospy.loginfo("everything is fine :)")



        

        