#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
from __future__ import annotations

import rospy
import numpy as np
import re
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from path_finder import PathFinder
from path_follower import PathFollower
from formation_builder.msg import Formation, GoalPose, Trajectory
from geometry_msgs.msg import Pose
from visualization import fb_visualizer


class CentralController:
    def __init__(self) -> None:
        rospy.init_node('CentralController')

        rospy.loginfo("[CController] Waiting for Services")
        rospy.wait_for_service('/formation_builder/pixel_to_world')
        rospy.wait_for_service('/formation_builder/world_to_pixel')
        rospy.loginfo("[CController] transformation services is running!")

        topics = rospy.get_published_topics()
        self.unique_mir_ids : set[int] = set()
        pattern = r"/mir(\d+)/"
        for topic in topics:
            match_string = re.search(pattern, str(topic))
            if match_string:
                robot_id = int(match_string.group(1))
                self.unique_mir_ids.add(robot_id)

        rospy.loginfo(f"[CController] Registered {len(self.unique_mir_ids)} mir bots, with the IDs: {self.unique_mir_ids}")


        self.path_finders : dict[int, PathFinder] = {id: PathFinder(id) for id in self.unique_mir_ids}
        #self.path_followers:dict[int, PathFollower] = {id: PathFollower(id) for id in self.unique_mir_ids}

        self.formation_subscriber : rospy.Subscriber = rospy.Subscriber("/formation_builder/formation", Formation, self.build_formation)
        self.map_subscriber : rospy.Subscriber = rospy.Subscriber("formation_builder/map", Image, self.map_callback)
        self.trajectory_publisher : rospy.Publisher = rospy.Publisher('formation_builder/trajectory', Trajectory, queue_size=10, latch=True)

        self.grid : np.ndarray | None = None
        self.cv_bridge : CvBridge = CvBridge()
        return None
    

    def build_formation(self, formation : Formation) -> None:
        if self.grid is None:
            rospy.logwarn("[CController] Failed to plan paths since there is no map data.")
            return None
        if formation.goal_poses is None:
            rospy.logwarn("[CController] Received an empty formation request.")
            return None
        start_time : float = time.time()
        rospy.loginfo(f"[CController] Received a planning request for {len(formation.goal_poses)} robots.")
        
        planned_trajectories : list[Trajectory] = []
        for gp in formation.goal_poses:
            goal_pose : GoalPose = gp
            if goal_pose.planner_id not in self.path_finders.keys():
                rospy.logwarn(f"[CController] Received a request for robot {goal_pose.planner_id} but this robot seems to not exist.")
                continue
            if goal_pose.goal is None:
                rospy.logwarn(f"[CController] Received a request for robot {goal_pose.planner_id} the goal position is none.")
                continue
            planned_trajectory : Trajectory | None = self.path_finders[goal_pose.planner_id].search_path(self.grid, goal_pose, planned_trajectories)
            if planned_trajectory is not None:
                planned_trajectories.append(planned_trajectory)

        

        rospy.loginfo(f"[CController] ------------ Planning done! ({time.time()-start_time:.3f}s) ------------ ")
        for trajectory in planned_trajectories:
            self.trajectory_publisher.publish(trajectory)

        fb_visualizer.show_live_path(planned_trajectories)
        end_time = time.time()
        rospy.loginfo(f"[CController] ################## Done! ({end_time-start_time:.3f}s) ################## ")

        return None
    
    
    def map_to_grid(self, map : Image) -> np.ndarray:
        img = self.cv_bridge.imgmsg_to_cv2(map, desired_encoding="mono8")
        grid : np.ndarray = np.array(img) // 255
        return grid
    

    def map_callback(self, img_msg : Image) -> None:
        grid : np.ndarray = self.map_to_grid(img_msg)
        #todo: check if obstacles interfere with planned routes; replan if necessary
        self.grid = grid
        return None


    
    def generate_formation_position(self, id : int, goal : tuple[float, float], size : float = 1.0) -> GoalPose:
        rospy.loginfo(f"[CController] Generating request for robot {id}")
        request : GoalPose = GoalPose()
        request.planner_id = id
        request.goal = Pose()
        request.goal.position.x, request.goal.position.y = goal
        request.robot_size = size
        return request
    



if __name__ == '__main__':
    central_controller: CentralController = CentralController()

    test_formation : Formation = Formation()
    test_formation.goal_poses = []
    goal_positions : list[tuple[float, float]] = [(50, 20), (50, 19), (50, 18), (51, 20)]
    for index, goal_position in enumerate(goal_positions):
        test_formation.goal_poses.append(central_controller.generate_formation_position(index+1, goal_position, size=1.0))

    rate : rospy.Rate = rospy.Rate(1)
    rate.sleep()

    temp_pub : rospy.Publisher = rospy.Publisher("/formation_builder/formation", Formation, queue_size=10, latch=True)
    temp_pub.publish(test_formation)

    rospy.spin()