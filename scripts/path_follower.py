#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

from __future__ import annotations

import time
import rospy
import actionlib
import mbf_msgs.msg as mbf_msgs
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray

from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, Pose
from formation_builder.msg import Trajectory, Waypoint
from tf.transformations import euler_from_quaternion

class PathFollower:
    def __init__(self, robot_id) -> None:
        #rospy.init_node(f'path_follower_{robot_id}')
        rospy.loginfo(f"[Follower {robot_id}] Initializing!")
        self.robot_id : int = robot_id
        self.trajectory : Trajectory | None = None
        #self.trajectory_start_time :rospy.Time = rospy.Time.now()
        self.trajectory_start_time : float = time.time()
        self.robot_pose : Pose | None = None

        self.max_linear_speed : float = 1.2
        self.max_angular_speed : float = 1.0
        self.lookahead_distance : float = 1.0
        self.robot_yaw : float = 0.0

        self.k_linear : float = 1.0
        self.k_angular:float = 2.0


        rospy.Subscriber('formation_builder/trajectory', Trajectory, self.trajectory_update)
        rospy.Subscriber(f'/mir{self.robot_id}/mir_pose_simple', Pose, self.update_pose)

        self.goal_publisher = rospy.Publisher(f'/mir{self.robot_id}/move_base_simple/goal', PoseStamped, queue_size=10)
        self.cmd_publisher = rospy.Publisher(f'/mir{self.robot_id}/cmd_vel', Twist, queue_size=10)
        self.reached_waypoints : int = 0
        self.target_waypoint : Waypoint | None = None

        #rospy.loginfo(f"[Follower {robot_id}] Waiting to connect to movement client...")
        #self.movement_client : actionlib.SimpleActionClient = actionlib.SimpleActionClient(f"/mir{self.robot_id}/move_base_flex/move_base", mbf_msgs.MoveBaseAction)
        #rospy.loginfo(f"[Follower {robot_id}] Connected!")

        self.follow_trajectory()
        rospy.spin()
        return None
    

    def update_pose(self, pose: Pose) -> None:
        self.robot_pose = pose
        (_, _, self.robot_yaw) = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        return None


    def get_distance(self, robot_pose : Pose, waypoint : Waypoint) -> float:
        return np.hypot(robot_pose.position.x - waypoint.world_position.position.x, robot_pose.position.y - waypoint.world_position.position.y)


    def get_angle(self, robot_pose : Pose, waypoint : Waypoint) -> float:
        return np.arctan2(waypoint.world_position.position.y - robot_pose.position.y, waypoint.world_position.position.x - robot_pose.position.x)


    def update_target_point(self) -> None:
        if self.trajectory is None or self.robot_pose is None or self.trajectory.path is None:
            rospy.logwarn(f"[Follower {self.robot_id}] Can't update the target point since Planner is not initialized")
            return None
        
        if self.trajectory.planner_id == 1:
        
            marker_pub = rospy.Publisher('/formation_builder/pure_pursuit', Marker, queue_size=10, latch=True)
            marker: Marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "formation_builder"
            marker.id = self.trajectory.planner_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            #marker.lifetime = rospy.Duration(0, int(1_000_000_000 / time_factor))
            marker.lifetime = rospy.Duration(10, 0)
            marker.pose.orientation.w = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.position = self.target_waypoint.world_position.position
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            point : Point = Point()
            point.x = self.target_waypoint.world_position.position.x
            point.y = self.target_waypoint.world_position.position.y
            point.z = self.target_waypoint.world_position.position.z
            marker.points = [point]
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker_pub.publish(marker)


        
        if self.target_waypoint is None:
            self.target_waypoint = self.trajectory.start_waypoint
        if self.target_waypoint == self.trajectory.goal_waypoint:
            return None
        distance_to_target : float = self.get_distance(self.robot_pose, self.target_waypoint)
        while distance_to_target < self.lookahead_distance and len(self.trajectory.path) > self.reached_waypoints:
            #if rospy.Time.now().secs + (distance_to_target / self.max_linear_speed) < self.target_waypoint.occupied_from + self.trajectory_start_time.secs:
            if time.time() - self.trajectory_start_time < self.trajectory.path[self.reached_waypoints].occupied_from:
                rospy.loginfo(f"[Follower {self.robot_id}] is too fast. expected to arrive at target at {time.time() + (distance_to_target / self.max_linear_speed)} but waypoint is occupied from {self.target_waypoint.occupied_from + self.trajectory_start_time}")
                break
            self.reached_waypoints += 1
            self.target_waypoint = self.trajectory.path[self.reached_waypoints]
            
            distance_to_target : float = self.get_distance(self.robot_pose, self.target_waypoint)
        return None


    def trajectory_update(self, trajectory : Trajectory) -> None:
        if trajectory.planner_id != self.robot_id:
            return None
        rospy.loginfo(f"[Follower {self.robot_id}] Received a new Trajectory.")
        self.trajectory = trajectory
        self.reached_waypoints = 0
        self.target_waypoint = trajectory.start_waypoint
        self.trajectory_start_time = time.time()
        self.follow_trajectory()
        return None
    

    def control_speeds(self, distance_to_target, steering_angle)->tuple[float, float]:
        linear_speed = min(self.max_linear_speed, self.k_linear * distance_to_target)
        angular_speed = self.k_angular * steering_angle

        return linear_speed, angular_speed




    def follow_trajectory(self) -> None:
        rate : rospy.Rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.robot_pose is None or self.target_waypoint is None:
                #rospy.logwarn(f"[Follower {self.robot_id}] Waiting for initialization")
                continue
            self.update_target_point()
            angle_to_target : float = self.get_angle(self.robot_pose, self.target_waypoint)
            steering_angle : float = angle_to_target - self.robot_yaw
            if steering_angle > np.pi:
                steering_angle -= 2 * np.pi
            elif steering_angle < -np.pi:
                steering_angle += 2 * np.pi

            distance_to_target = self.get_distance(self.robot_pose, self.target_waypoint)
            linear_speed, angular_speed = self.control_speeds(distance_to_target, steering_angle)

            twist_msg = Twist()
            twist_msg.linear.x = linear_speed
            twist_msg.angular.z = min(self.max_angular_speed, max(-self.max_angular_speed, angular_speed))
            if abs(steering_angle) > np.pi * 0.3: # stop driving forward if turn is too hard #todo: make this adaptiv
                twist_msg.linear.x = 0.0
            self.cmd_publisher.publish(twist_msg)

            rate.sleep()
            continue




            twist_msg = Twist()
            #twist_msg.linear.x = self.max_linear_speed
            twist_msg.linear.x = min(self.max_angular_speed, self.get_distance(self.robot_pose, self.target_waypoint))


            steering_direction : float = np.sign(steering_angle)
            steering_velocity : float = min(steering_angle ** 2, self.max_angular_speed)
            
            if steering_velocity > 0.5: #todo: smarter system for hard turns
                twist_msg.linear.x = 0


            twist_msg.angular.z = steering_velocity * steering_direction
            self.cmd_publisher.publish(twist_msg)

            rate.sleep()
            continue



            
            
            if abs(steering_angle) > 0.1:
                twist_msg = Twist()
                twist_msg.angular.z = self.max_angular_speed if steering_angle > 0 else -self.max_angular_speed
                self.cmd_publisher.publish(twist_msg)
            else:
                twist_msg = Twist()
                twist_msg.linear.x = self.max_linear_speed
                self.cmd_publisher.publish(twist_msg)
            rate.sleep()
        return None





        return None
        rate : rospy.Rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.move_to_waypoint()
            rate.sleep()
        return None
    

    def move_to_waypoint(self) -> None:
        return None
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
            self.goal_publisher.publish(pose)


    
    
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