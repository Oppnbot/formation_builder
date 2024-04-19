#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
from __future__ import annotations

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import re
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from formation_builder.msg import GridMap
from std_srvs.srv import Empty


class CostMapReader:
    def __init__(self) -> None:
        rospy.init_node('CostMapReader')

        topics = rospy.get_published_topics()


        self.unique_mir_ids : set[int] = set()
        pattern = r"/mir(\d+)/"
        for topic in topics:
            match_string = re.search(pattern, str(topic))
            if match_string:
                robot_id = int(match_string.group(1))
                self.unique_mir_ids.add(robot_id)

        #rospy.Subscriber("/mir1/move_base_flex/global_costmap/costmap", OccupancyGrid, self.print_msg)
        self.costmap_subsribers : list[rospy.Subscriber] = [rospy.Subscriber(f"/mir{robot_id}/move_base_flex/global_costmap/costmap", OccupancyGrid, self.read_costmap, callback_args=robot_id) for robot_id in self.unique_mir_ids]
        #self.costmap_subsribers : list[rospy.Subscriber] = [rospy.Subscriber(f"/mir{robot_id}/move_base_flex/global_costmap/costmap", OccupancyGrid, self.read_costmap) for robot_id in self.unique_mir_ids]
        self.clear_services : list[rospy.ServiceProxy] = [rospy.ServiceProxy(f"/mir{robot_id}/move_base_flex/clear_costmaps", Empty) for robot_id in self.unique_mir_ids]

        self.costmaps : dict[int, OccupancyGrid] = {}
        rospy.Timer(rospy.Duration(10, 0), self.clear_costmaps, oneshot=False)
        rospy.Timer(rospy.Duration(5, 0), self.merge_costmaps, oneshot=False)
        self.merged_costmap_publisher : rospy.Publisher = rospy.Publisher("/formation_builder/merged_costmap", OccupancyGrid, queue_size=5, latch=True)
        return None
    

# mir1/move_base_flex/clear_costmaps

    def print_msg(self, _) -> None:
        rospy.logerr("new data")
        return None

    def merge_costmaps(self, _) -> OccupancyGrid | None:
        if len(self.costmaps.values()) < 1:
            rospy.loginfo("No Costmaps Available")
            return None     
        height : int = list(self.costmaps.values())[0].info.height
        width : int = list(self.costmaps.values())[0].info.width

        merged_data : np.ndarray = np.zeros_like(height*width)

        for costmap in self.costmaps.values():
            costmap_data = np.array(costmap.data, dtype=np.int8)
            costmap_data = costmap_data.reshape(-1, 1)
            merged_data = np.maximum(merged_data, costmap_data)
        merged_costmap = OccupancyGrid()
        merged_costmap.header = list(self.costmaps.values())[-1].header
        merged_costmap.info = list(self.costmaps.values())[-1].info
        merged_costmap.data = merged_data.flatten().tolist()
        rospy.loginfo("Publishing Merged Map")
        self.merged_costmap_publisher.publish(merged_costmap)
        return merged_costmap
    

    def read_costmap(self, costmap : OccupancyGrid, robot_id : int = -1) -> None:
        if robot_id == -1:
            rospy.logwarn("robot id is invalid")
            return None
        rospy.loginfo(f"Received a Map for Robot {robot_id}")
        self.costmaps[robot_id] = costmap
        #todo: remove robots and robot trails from costmap
        return None


    def clear_costmaps(self, _) -> None:
        rospy.loginfo("Clearing Costmaps...")
        for clear_service in self.clear_services:
            clear_service()
        return None

if __name__== '__main__':
    fb_map_reader : CostMapReader = CostMapReader()
    rospy.spin()
