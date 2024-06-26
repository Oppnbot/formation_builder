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
from formation_builder.msg import Formation, GoalPose, Trajectory, Trajectories, FollowerFeedback
from geometry_msgs.msg import Pose


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
        self.current_formation : Formation | None = None
        self.grid : np.ndarray | None = None
        self.cv_bridge : CvBridge = CvBridge()

        self.follower_feedback_subscriber : rospy.Subscriber = rospy.Subscriber('formation_builder/follower_status', FollowerFeedback, self.follower_feedback)
        self.formation_subscriber : rospy.Subscriber = rospy.Subscriber("/formation_builder/formation", Formation, self.build_formation)
        self.map_subscriber : rospy.Subscriber = rospy.Subscriber("formation_builder/map", Image, self.map_callback)
        self.trajectory_publisher : rospy.Publisher = rospy.Publisher('formation_builder/trajectories', Trajectories, queue_size=10, latch=True)
        return None
    

    def follower_feedback(self, feedback : FollowerFeedback) -> None:
        if self.current_formation is None:
            return None
        if feedback.status == feedback.LOST_WAYPOINT:
            rospy.loginfo(f"[CController] Replanning because Robot {feedback.robot_id} lost its waypoint.")
            self.build_formation(self.current_formation)
        if feedback.status == feedback.OUTSIDE_RESERVED_AREA:
            rospy.loginfo(f"[CController] Replanning because Robot {feedback.robot_id} left its reserved area.")
            self.build_formation(self.current_formation)
        if feedback.status == feedback.PATH_BLOCKED:
            rospy.loginfo(f"[CController] Replanning because Robot {feedback.robot_id} Path is blocked.")
            self.build_formation(self.current_formation)
        return None
    
    def check_priorities(self) -> bool:
        if self.current_formation is None or self.current_formation.goal_poses is None:
            rospy.logerr(f"[CController] Can't check priorities since goal poses or current formation are None")
            return False
        prio_counter : dict[int, int] = {}
        for goal_pose in self.current_formation.goal_poses:
            if goal_pose.priority is None:
                rospy.loginfo(f"[CController] Priority of Robot {goal_pose.planner_id} is None. Will reset priorities")
                return False
            prio_counter[goal_pose.priority] = prio_counter.get(goal_pose.priority, 0) + 1

        for prio, count in prio_counter.items():
            if count != 1:
                rospy.logwarn(f"[CController] Priority {prio} is assigned to multiple ({count}) robots. Will reset priorities")
                return False
        return True

    def initialize_priorities(self) -> None:
        if self.current_formation is None or self.current_formation.goal_poses is None:
            rospy.logwarn(f"[CController] Can't initialize priorities since goal poses or current formation are None")
            return None
        for index, goal_pose in enumerate(self.current_formation.goal_poses):
            goal_pose.priority = index + 1
        return None
    
    
    def reassign_priorities(self, robot_id : int) -> bool:
        if self.current_formation is None or self.current_formation.goal_poses is None:
            rospy.logerr(f"[CController] Can't change Priority of Robot {robot_id} since goal poses or current formation are None")
            return False
        # set failed planner to highest priority
        prio_failed_robot : int = -1
        for goal_pose in self.current_formation.goal_poses:
            if goal_pose.planner_id == robot_id:
                prio_failed_robot = goal_pose.priority
                goal_pose.priority = 1
                break
        if prio_failed_robot == -1:
            rospy.logerr(f"[CController] Can't change Priority of Robot {robot_id} since planner id seems to not exist in the current formation.")
            return False
        # adjust priorities of remaining robots
        for goal_pose in self.current_formation.goal_poses:
            if goal_pose.planner_id != robot_id and goal_pose.priority < prio_failed_robot:
                goal_pose.priority += 1
        return True


    def build_formation(self, formation : Formation) -> None:
        if self.grid is None:
            rospy.logwarn("[CController] Failed to plan paths since there is no map data.")
            return None
        if formation.goal_poses is None:
            rospy.logwarn("[CController] Received an empty formation request.")
            return None
        self.current_formation = formation
        start_time : float = time.time()
        rospy.loginfo(f"[CController] Received a planning request for {len(formation.goal_poses)} robots.")

        while not self.check_priorities():
            self.initialize_priorities()
        else:
            rospy.loginfo("[CController] Priorities are set correctly.")
        
        planned_trajectories : list[Trajectory] = []
        failed_planner : int | None = None
        formation.goal_poses.sort(key=lambda x: x.priority)
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
            else:
                failed_planner = goal_pose.planner_id
                rospy.logwarn(f"[CController] Planner failed {failed_planner}")
                break

        

        if failed_planner is not None:
            rospy.logwarn(f"[CController] ------------ Planning failed! ({time.time()-start_time:.3f}s) ------------ ")
            for goal_pose in formation.goal_poses:
                if goal_pose.planner_id == failed_planner:
                    if goal_pose.priority == 1:
                        rospy.logerr(f"[CController] Planner {failed_planner} failed with highest priority. There is no valid solution.")
                        return None
                    break
            if self.reassign_priorities(failed_planner):
                rospy.loginfo("[CController] Retrying with reordered priorites...")
                self.build_formation(self.current_formation)
            else:
                rospy.logerr(f"[CController] Failed to replan with reordered priorities.")
            return None

        rospy.loginfo(f"[CController] ------------ Planning done! ({time.time()-start_time:.3f}s) ------------ ")
        trajectories : Trajectories = Trajectories()
        trajectories.trajectories = [trajectory for trajectory in planned_trajectories]
        trajectories.timestamp = time.time()
        self.trajectory_publisher.publish(trajectories)

        #fb_visualizer.show_live_path(planned_trajectories)
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


    
    def generate_formation_position(self, id : int, goal : tuple[float, float, float], size : float = 1.0) -> GoalPose:
        rospy.loginfo(f"[CController] Generating request for robot {id}")
        request : GoalPose = GoalPose()
        request.planner_id = id
        request.goal = Pose()
        request.goal.position.x, request.goal.position.y = goal[0], goal[1]
        request.goal.orientation.z = goal[2]
        request.robot_size = size
        return request
    



if __name__ == '__main__':
    central_controller: CentralController = CentralController()

    test_formation : Formation = Formation()
    test_formation.goal_poses = []
    east : float = 0.0
    west : float = np.pi
    north : float = np.pi/2
    south : float = -np.pi/2

    #goal_positions : list[tuple[float, float]] = [(50, 20), (50, 19), (30, 18), (51.1, 20)]

    formation_square = [(36, 37, north), (33, 39.5, north) , (36, 39.5, north), (33, 37, north)]
    formation_line_north = [(31, 38, north), (33.5, 38, north), (36, 38, north), (38.5, 38, north)]
    formation_line_north_scrambeled = [(33.5, 38, north),(38.5, 38, north), (36, 38, north), (31, 38, north)]
    formation_line_north_reversed = [ (38.5, 38, north), (36, 38, north),(33.5, 38, north), (31, 38, north)]
    formation_line_south = [(31, 34, north), (33.5, 34, north), (36, 34, north), (38.5, 34, north)]
    goal_positions : list[tuple[float, float, float]] = formation_line_south

    for index, goal_position in enumerate(goal_positions):
        test_formation.goal_poses.append(central_controller.generate_formation_position(index+1, goal_position, size=1.0))

    rate : rospy.Rate = rospy.Rate(1)
    rate.sleep()

    temp_pub : rospy.Publisher = rospy.Publisher("/formation_builder/formation", Formation, queue_size=10, latch=True)
    temp_pub.publish(test_formation)

    rospy.spin()