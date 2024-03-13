#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

from __future__ import annotations

import time
import rospy
import actionlib
import mbf_msgs.msg as mbf_msgs
import numpy as np

from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, Pose
from formation_builder.msg import Trajectory, Waypoint


class PathFollower:
    def __init__(self, robot_id) -> None:
        #rospy.init_node(f'path_follower_{robot_id}')
        rospy.loginfo(f"[Follower {robot_id}] Initializing!")
        self.robot_id : int = robot_id
        self.trajectory : Trajectory | None = None
        self.trajectory_start_time : float = time.time()
        self.robot_pose : Pose | None = None

        rospy.Subscriber('formation_builder/trajectory', Trajectory, self.trajectory_update)
        rospy.Subscriber(f'/mir{self.robot_id}/mir_pose_simple', Pose, self.update_pose)

        self.publisher = rospy.Publisher(f'/mir{self.robot_id}/move_base_simple/goal', PoseStamped, queue_size=10)
        self.reached_waypoints : int = 0

        #rospy.loginfo(f"[Follower {robot_id}] Waiting to connect to movement client...")
        #self.movement_client : actionlib.SimpleActionClient = actionlib.SimpleActionClient(f"/mir{self.robot_id}/move_base_flex/move_base", mbf_msgs.MoveBaseAction)
        #rospy.loginfo(f"[Follower {robot_id}] Connected!")

        self.follow_trajectory()
        rospy.spin()
        return None
    

    def update_pose(self, pose: Pose) -> None:
        self.robot_pose = pose
        return None
    

    def trajectory_update(self, trajectory : Trajectory) -> None:
        if trajectory.planner_id != self.robot_id:
            return None
        rospy.loginfo(f"[Follower {self.robot_id}] Received a new Trajectory.")
        self.trajectory = trajectory
        self.reached_waypoints = 0
        self.trajectory_start_time = time.time()
        return None
    

    def follow_trajectory(self) -> None:
        rate : rospy.Rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.move_to_waypoint()
            rate.sleep()
        return None
    

    def move_to_waypoint(self) -> None:
        if self.robot_pose is None:
            rospy.logwarn(f"[Follower {self.robot_id}] Pose is None.")
            return None
        if self.trajectory is None:
            rospy.logwarn(f"[Follower {self.robot_id}] Trajectory is None.")
            return None
        if self.trajectory.path is None:
            rospy.logwarn(f"[Follower {self.robot_id}] Path is None.")
            return None
        if self.reached_waypoints >= len(self.trajectory.path):
            rospy.logwarn(f"[Follower {self.robot_id}] Waiting for a new path...")
            return None
        
        current_waypoint : Waypoint = self.trajectory.path[self.reached_waypoints]
        dist_to_waypoint : float = np.abs(current_waypoint.world_position.position.x - self.robot_pose.position.x) + np.abs(current_waypoint.world_position.position.y - self.robot_pose.position.y)
        grid_size = 2.0 #! get this from map

        if dist_to_waypoint <= grid_size:
            waypoints : list[Waypoint] = self.trajectory.path
            if waypoints[self.reached_waypoints + 1].occupied_from < time.time() - self.trajectory_start_time:
                rospy.loginfo(f"[Follower {self.robot_id}] Reached Waypoint {self.reached_waypoints}")
                self.reached_waypoints += 1
                if self.reached_waypoints >= len(self.trajectory.path):
                    rospy.loginfo(f"[Follower {self.robot_id}] Reached it's goal.")
                    return None
            
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'map'
            pose.pose = current_waypoint.world_position
            pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
            self.publisher.publish(pose)


    
    
    #def move_to_waypoint(self, waypoint : Waypoint) -> bool:
    #    goal  : mbf_msgs.MoveBaseGoal = mbf_msgs.MoveBaseGoal()
    #    goal.target_pose.header.frame_id = "map"
    #    goal.target_pose.header.stamp = rospy.Time.now()
    #    goal.target_pose.pose = waypoint.world_position
    #    self.movement_client.send_goal(goal)
    #    self.movement_client.wait_for_result()
    #
    #    if self.movement_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
    #        rospy.loginfo("Reached Goal")
    #        return True
    #    else:
    #        rospy.loginfo("Failed to reach goal...")
    #        return False
    #    return False
            
if __name__ == '__main__':
    rospy.init_node("path_follower")
    planner_id = rospy.get_param('~robot_id', default=-1)
    path_follower : PathFollower = PathFollower(planner_id)