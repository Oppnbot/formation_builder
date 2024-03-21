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
        self.id : int = robot_id
        #self.scan_subscriber : rospy.Subscriber = rospy.Subscriber(f"/mir{robot_id}/scan", LaserScan, self.scanner_callback)
        self.pointcloud_publisher : rospy.Publisher = rospy.Publisher(f"/mir{robot_id}/scan_pc", PointCloud2, queue_size=10)
        self.front_scan : LaserScan | None = None
        self.back_scan : LaserScan | None = None
        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform("map", f"mir{robot_id}/front_laser_link", rospy.Time(0), rospy.Duration(3))
        self.tf_listener.waitForTransform("map", f"mir{robot_id}/back_laser_link", rospy.Time(0), rospy.Duration(3))
        self.scan_left_subscriber : rospy.Subscriber = rospy.Subscriber(f"/mir{robot_id}/f_scan", LaserScan, self.scanner_callback, 'front')
        self.scan_right_subscriber : rospy.Subscriber = rospy.Subscriber(f"/mir{robot_id}/b_scan", LaserScan, self.scanner_callback, 'back')
        self.transform = TransformStamped()
        self.laser_projection = LaserProjection()

        rospy.Subscriber(f'/mir{robot_id}/mir_pose_simple', Pose, self.update_pose)
        return None
    
    def update_pose(self, pose: Pose) -> None:
        self.robot_pose = pose
        (_, _, self.robot_yaw) = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        return None
    
    def scanner_callback(self, scan: LaserScan, side : str) -> None:
        if self.id != 1:
            return None

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


        data : list[list[float]] = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(cloud_transformed, remove_nans=True)


        is_colliding: bool = False
        for point in data:
            if abs(point[1]) < 0.1 and 0 < point[0] < 1:
                is_colliding = True
        if is_colliding:
            rospy.logerr("COLLISION INCOMING!!!")
        else:
            rospy.loginfo("everything is fine :)")



        

        


        return None
        frame_name : str = scan.header.frame_id.lower()
        if "front" in frame_name:
            self.front_scan = scan
            self.convert_scan_to_pointcloud(scan, (0.0, 0.0), 0.0)
        elif "back" in frame_name:
            self.front_back = scan
        else:
            rospy.logwarn(f"[SCANNER {self.id}]: Doesn't recognize {frame_name} since it does not contain 'front' or 'back'")
            return None
        
        return None
    
    def convert_scan_to_pointcloud(self, scan: LaserScan, scanner_offset: tuple[float, float], scanner_angle: float) -> None:
        return None
        point_cloud : PointCloud2 = PointCloud2()
        point_cloud.header = scan.header

        starting_angle : float = scan.angle_min
        stopping_angle : float = scan.angle_max
        angle_increment: float = scan.angle_increment
        point_cloud.data = []
        point_cloud.fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1), PointField('z', 8, PointField.FLOAT32, 1)]

        points : list[list[float]] = []
        for index, distance in enumerate(scan.ranges):
            if distance == float('inf'):
                continue
            angle : float = starting_angle + index * angle_increment
            x : float = distance * math.cos(angle)
            y : float = distance * math.sin(angle)
            z : float = 0.0
            points.append([x, y, z])
        
        point_cloud.is_bigendian = False
        point_cloud.point_step = 12
        point_cloud.is_dense = True
        point_cloud.width = len(points)
        point_cloud.height = 1
        point_cloud.data = np.array(points, dtype=np.float32).tostring()

        self.pointcloud_publisher.publish(point_cloud)
        return None
