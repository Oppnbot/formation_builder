#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
from __future__ import annotations

import rospy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import numpy as np
import re
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

        self.costmap_subscribers : list[rospy.Subscriber] = [rospy.Subscriber(f"/mir{robot_id}/move_base_flex/global_costmap/costmap", OccupancyGrid, self.read_costmap, callback_args=robot_id) for robot_id in self.unique_mir_ids]
        self.costmap_update_subscribers : list[rospy.Subscriber] = [rospy.Subscriber(f"/mir{robot_id}/move_base_flex/global_costmap/costmap_updates", OccupancyGridUpdate, self.update_costmap, callback_args=robot_id) for robot_id in self.unique_mir_ids]
        self.clear_services : list[rospy.ServiceProxy] = [rospy.ServiceProxy(f"/mir{robot_id}/move_base_flex/clear_costmaps", Empty) for robot_id in self.unique_mir_ids]

        self.costmaps : dict[int, OccupancyGrid] = {}
        self.costmap_data : dict[int, np.ndarray] = {}
        rospy.Timer(rospy.Duration(3, 0), self.clear_costmaps, oneshot=False)
        rospy.Timer(rospy.Duration(1, 0), self.merge_costmaps, oneshot=False)
        self.merged_costmap_publisher : rospy.Publisher = rospy.Publisher("/formation_builder/merged_costmap", OccupancyGrid, queue_size=5, latch=True)
        return None
    

    def merge_costmaps(self, _) -> OccupancyGrid | None:
        if len(self.costmaps.values()) < 1:
            rospy.loginfo("[CMap Reader]: No Costmaps Available")
            return None
        # convert vector to a numpy array for easier merging of different sized maps
        height : int = 0
        width : int = 0
        for costmap in self.costmaps.values():
            if costmap.info.width > width:
                width = costmap.info.width
            if costmap.info.height > height:
                height = costmap.info.height
        
        merged_data : np.ndarray = np.zeros((height, width))
        for costmap_data in self.costmap_data.values():
            merged_data = np.maximum(merged_data, costmap_data)

        merged_costmap : OccupancyGrid = OccupancyGrid()
        merged_costmap.info = list(self.costmaps.values())[-1].info
        merged_costmap.info.height = height
        merged_costmap.info.width = width

        merged_costmap.data = merged_data.flatten().astype(int).tolist()
        rospy.loginfo("[CMap Reader]: Publishing Merged Costmap...")
        self.merged_costmap_publisher.publish(merged_costmap)
        return None


    def update_costmap(self, costmap_update: OccupancyGridUpdate, robot_id : int = -1) -> None:
        # Costmap Update. Does trigger regularly but only contains data from robots proximity. have to merge it with the initial costmap
        if robot_id not in self.costmaps:
            rospy.logwarn(f"[CMap Reader]: Robot ID {robot_id} is invalid; May not be fully initialized yet")
            return None
        self.costmaps[robot_id].data = costmap_update.data        
        self.costmap_data[robot_id][costmap_update.y:costmap_update.y+costmap_update.height, costmap_update.x:costmap_update.x+costmap_update.width] = np.reshape(costmap_update.data, (costmap_update.height, costmap_update.width))
        #todo: remove robots and robot trails from costmap
        return None
    

    def read_costmap(self, costmap : OccupancyGrid, robot_id : int = -1) -> None:
        # Initial Costmap Init. Does only trigger once so we have to update the costmap manually using update_costmap()
        if robot_id == -1:
            rospy.logwarn(f"[CMap Reader]: Robot ID {robot_id} is invalid")
            return None
        rospy.loginfo(f"[CMap Reader]: Received a Map for Robot {robot_id}")
        self.costmaps[robot_id] = costmap
        costmap_data : np.ndarray = np.reshape(costmap.data, (costmap.info.height, costmap.info.width))
        self.costmap_data[robot_id] = costmap_data
        #todo: remove robots and robot trails from costmap
        return None


    def clear_costmaps(self, _) -> None:
        rospy.loginfo("[CMap Reader]: Clearing Costmaps...")
        for clear_service in self.clear_services:
            clear_service()
        return None


if __name__== '__main__':
    fb_map_reader : CostMapReader = CostMapReader()
    rospy.spin()
