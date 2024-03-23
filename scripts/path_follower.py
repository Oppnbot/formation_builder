#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-


from __future__ import annotations

import time
import rospy
import actionlib
import mbf_msgs.msg as mbf_msgs
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as laser_geometry
from laser_scanner import LaserScanner


from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, Pose
from formation_builder.msg import Trajectory, Waypoint
from tf.transformations import euler_from_quaternion


class PathFollower:
    def __init__(self, robot_id:int) -> None:
        #rospy.init_node(f'path_follower_{robot_id}')
        rospy.loginfo(f"[Follower {robot_id}] Initializing!")
        self.robot_id : int = robot_id
        self.trajectory : Trajectory | None = None
        #self.trajectory_start_time :rospy.Time = rospy.Time.now()
        self.trajectory_start_time : float = time.time()
        self.robot_pose : Pose | None = None

        # ---- Config Zone ----
        self.lookahead_distance : float = 1.0   # higher distance -> smoother curves when driving but might leave path.
        self.max_linear_speed : float = 1.2     # [m/s] max driving speed
        self.max_angular_speed : float = 1.0    # [rad/s] max rotation speed
        
        self.goal_tolerance : float = 0.05 # [m] distance at which the goal position is considered to be reached
        self.rotation_tolerance : float = 0.002 # [rad] angle at which the goal rotation is considered to be reached
        self.slowdown_angle : float = 0.25 # [rad] angle at which the slowdown begins. might take longer to reach the desired orientation but will allow for higher precision
        
        self.slowdown_x : float = 3.0 # [m] defines a boxes x-axis that causes slowdowns to the robots speed if objects enter it
        self.slowdown_y : float = 0.7 # [m] defines a boxes y-axis that causes slowdowns to the robots speed if objects enter it
        self.stopping_x: float = 1.1 # [m]defines a box that x-axis causes a stop to the robots speed if objects enter it
        self.stopping_y : float = 0.7 # [m] defines a box that y-axis causes a stop to the robots speed if objects enter it
        self.robot_size_x : float = 0.9 # [m] robot size along x-axis. will igonore laser scans values within this range
        self.robot_size_y : float = 0.6 # [m] robot size along y-axis. will igonore laser scans values within this range
        # ---- End Config ----
        
        self.upper_linear_limit : float = self.max_linear_speed
        self.lower_linear_limit : float = -self.max_linear_speed
        self.upper_rotation_limit : float = self.max_angular_speed
        self.lower_rotation_limit : float = -self.max_angular_speed

        self.robot_yaw : float = 0.0
        self.k_linear : float = 1.0 # higher value -> faster linear movement value > 1.0 might lead to start/stop behaviour
        self.k_angular : float = 2.0 # higher value -> faster turnings
        
        self.scanner : LaserScanner = LaserScanner(self.robot_id)
        rate : rospy.Rate = rospy.Rate(100)
        while not self.scanner.initialized:
            rate.sleep()
        rospy.Subscriber('formation_builder/trajectory', Trajectory, self.trajectory_update)
        rospy.Subscriber(f'/mir{self.robot_id}/mir_pose_simple', Pose, self.update_pose)
        rospy.Subscriber(f'/mir{self.robot_id}/scan', LaserScan, self.safety_limit_update)

        self.goal_publisher = rospy.Publisher(f'/mir{self.robot_id}/move_base_simple/goal', PoseStamped, queue_size=10)
        self.cmd_publisher = rospy.Publisher(f'/mir{self.robot_id}/cmd_vel', Twist, queue_size=10)
        self.reached_waypoints : int = 0
        self.target_waypoint : Waypoint | None = None

        

        #rospy.loginfo(f"[Follower {robot_id}] Waiting to connect to movement client...")
        #self.movement_client : actionlib.SimpleActionClient = actionlib.SimpleActionClient(f"/mir{self.robot_id}/move_base_flex/move_base", mbf_msgs.MoveBaseAction)
        #rospy.loginfo(f"[Follower {robot_id}] Connected!")

        #self.follow_trajectory()
        self.stop_robot() #for safety reasons, dont remove this
        rospy.spin()
        return None
    

    def safety_limit_update(self, _ :LaserScan) -> None:

        _upper_linear_limit : float = self.max_linear_speed
        _lower_linear_limit : float = -self.max_linear_speed
        _upper_rotation_limit : float = self.max_angular_speed
        _lower_rotation_limit : float = -self.max_angular_speed

        laser_points : list[list[float]] = self.scanner.get_laser_points()
        is_ignoring : bool = False
        is_colliding : bool = False
        is_slowdown : bool = False
        crit_radius : float = np.hypot(self.robot_size_x / 2, self.robot_size_y / 2) * 1.05 # check rotational collision when object gets this close. multiplication factor for safety margin
        for point in laser_points:
            #* INVALID DATA FILTERING
            # Ignore Points that are inside our robot. this may happen if the laser scanner recognizes robot parts as obstacle
            if self.robot_size_x / 2 > abs(point[0]) and self.robot_size_y / 2 > abs(point[1]):
                is_ignoring = True
                continue
            #* LINEAR COLLISION AVOIDANCE
            # Check for very close obstacles -> stop
            if abs(point[0]) < self.stopping_x / 2 and abs(point[1]) < self.robot_size_y / 2:
                is_colliding = True
                if point[0] > 0:
                    _upper_linear_limit = 0.0
                else:
                    _lower_linear_limit = 0.0
            # Check for somewhat close obstacles -> slowdown
            elif abs(point[0]) < self.slowdown_x / 2 and abs(point[1]) < self.slowdown_y / 2:
                is_slowdown = True
                slope: float = self.max_linear_speed / ((self.slowdown_x - self.stopping_x) / 2)
                offset : float = -slope * self.stopping_x / 2

                x_vel : float = (slope * abs(point[0]) + offset)
                x_vel = min(x_vel, self.max_linear_speed)
                x_vel = max(x_vel, 0.05 * self.max_linear_speed) # define min velocity
                
                if point[0] > 0:
                    _upper_linear_limit = min(_upper_linear_limit, x_vel)
                else:
                    _lower_linear_limit = max(_lower_linear_limit, -x_vel)

            #* ROTATIONAL DATA FILTERING
            dist_from_robot: float = np.hypot(point[0], point[1])
            if dist_from_robot < crit_radius:
                #* CHECK FOR TOTAL STOP
                # check if object is in critical distance and we need to block all rotations that may lead to a collision
                is_clockwise_collision: bool = (np.sign(point[0]) == np.sign(point[1]))
                if abs(point[1]) < self.stopping_y / 2 and point[0] < self.stopping_x / 2:
                    # critical zone in the middle of the robot where both rotation directions may lead to a collsion
                    if abs(point[0]) < 0.1 * self.robot_size_x:
                        _upper_rotation_limit = 0
                        _lower_rotation_limit = 0
                    ## collisions near the robots corner only block one rotational direction
                    if is_clockwise_collision:
                        _upper_rotation_limit = 0
                    else:
                        _lower_rotation_limit = 0
                #* CHECK FOR SLOWDOWN
                dist_to_robot_flank : float = max(0, abs(point[1]) - self.robot_size_y / 2)
                angle : float = np.arctan2(dist_to_robot_flank, point[0])
                
                #angle : float = np.arctan2(point[1] - self.robot_size_y * np.sign(point[1]), point[0])
                speed_factor : float = 1.0 - 2 * abs(0.5 - angle/np.pi)
                if is_clockwise_collision:
                    _upper_rotation_limit = min(_upper_rotation_limit, speed_factor * self.max_angular_speed)
                else:
                    _lower_rotation_limit = max(_lower_rotation_limit, -speed_factor * self.max_angular_speed)
          
                    
        #rospy.loginfo(f"lower {_lower_rotation_limit:3f}, upper {_upper_rotation_limit:.3f}")
        #if _upper_rotation_limit == 0 or _lower_rotation_limit == 0:
        #    rospy.logwarn(f"blocks directions c: {_lower_rotation_limit == 0} / cc: {_upper_rotation_limit == 0}")
        #else:
        #    rospy.loginfo("no rotational collision")    
    
        #rospy.loginfo(f"rotational limits: ")


        #if is_ignoring:
        #    rospy.logerr("invalid point is inside robot")
        #if is_colliding:
        #    rospy.logwarn(f"[FOLLOWER {self.robot_id}]: Stopping because collision is imminent!")
        #elif is_slowdown:
        #    rospy.logwarn(f"slowing down to {_upper_linear_limit:.3f} / {_lower_linear_limit:.3f}")
        #else:
        #    rospy.loginfo("no obstacle present")

        self.upper_linear_limit = _upper_linear_limit
        self.lower_linear_limit = _lower_linear_limit
        self.upper_rotation_limit = _upper_rotation_limit
        self.lower_rotation_limit = _lower_rotation_limit
            # Check if there are objects inside the slowdown area
            #if abs(point[1]) < self.slowdown_width / 2 and abs(point[0]) < self.slowdown_height / 2:     
        return None


    def stop_robot(self) -> None:
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_publisher.publish(twist_msg)
        return None
    

    def update_pose(self, pose: Pose) -> None:
        self.robot_pose = pose
        (_, _, self.robot_yaw) = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        return None


    def get_distance(self, robot_pose : Pose, waypoint : Waypoint) -> float:
        return np.hypot(robot_pose.position.x - waypoint.world_position.position.x, robot_pose.position.y - waypoint.world_position.position.y)


    def get_angle(self, robot_pose : Pose, waypoint : Waypoint) -> float:
        return np.arctan2(waypoint.world_position.position.y - robot_pose.position.y, waypoint.world_position.position.x - robot_pose.position.x)


    def update_target_point(self) -> Waypoint | None:
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
            return self.trajectory.goal_waypoint
        
        distance_to_target : float = self.get_distance(self.robot_pose, self.target_waypoint)
        while distance_to_target < self.lookahead_distance and len(self.trajectory.path) > self.reached_waypoints:
            #if rospy.Time.now().secs + (distance_to_target / self.max_linear_speed) < self.target_waypoint.occupied_from + self.trajectory_start_time.secs:
            if time.time() - self.trajectory_start_time < self.trajectory.path[self.reached_waypoints].occupied_from:
                #rospy.loginfo(f"[Follower {self.robot_id}] is too fast. expected to arrive at target at {time.time() + (distance_to_target / self.max_linear_speed)} but waypoint is occupied from {self.target_waypoint.occupied_from + self.trajectory_start_time}")
                break
            self.reached_waypoints += 1
            if len(self.trajectory.path) <= self.reached_waypoints:
                continue
            self.target_waypoint = self.trajectory.path[self.reached_waypoints]
            distance_to_target : float = self.get_distance(self.robot_pose, self.target_waypoint)
        return self.trajectory.path[self.reached_waypoints]


    def trajectory_update(self, trajectory : Trajectory) -> None:
        if trajectory.planner_id != self.robot_id:
            return None
        rospy.loginfo(f"[Follower {self.robot_id}] Received a new Trajectory.")
        self.trajectory = trajectory
        self.reached_waypoints = 0
        self.target_waypoint = trajectory.start_waypoint
        self.trajectory_start_time = time.time()
        rate : rospy.Rate = rospy.Rate(100)


        while not rospy.is_shutdown():
            if self.follow_trajectory():
                break
            rate.sleep()

        while not rospy.is_shutdown():
            if self.rotate(self.trajectory.goal_waypoint.world_position.orientation.z):
                break
            rate.sleep()
        
        rospy.loginfo(f"[Follower {self.robot_id}] Done!")
        self.stop_robot()
        return None
    

    def control_speeds(self, distance_to_target, steering_angle) -> tuple[float, float]:
        linear_speed = min(self.max_linear_speed, self.k_linear * distance_to_target)
        angular_speed = self.k_angular * steering_angle

        return linear_speed, angular_speed
    

    def get_min_angle(self, angle_1 : float, angle_2 : float) -> float:
        min_angle : float = angle_1 - angle_2
        min_angle -= 2*np.pi if min_angle > np.pi else 0.0
        min_angle += 2*np.pi if min_angle < -np.pi else 0.0
        return min_angle
    

    def rotate(self, target_rotation : float) -> bool:
        angle_error : float = self.get_min_angle(target_rotation, self.robot_yaw)
            
        #rospy.loginfo(f"angle error {angle_error}")

        rotation_direction : float = np.sign(angle_error)
        twist_msg = Twist()
        twist_msg.linear.x = 0.0

        if abs(angle_error) < self.rotation_tolerance:
            twist_msg.angular.z = 0.0
            self.cmd_publisher.publish(twist_msg)
            rospy.loginfo(f"[Follower {self.robot_id}] Reached the Goal Orientation with a tolerance of {abs(angle_error):.5f} rad")
            return True
        
        angular_speed : float = self.max_angular_speed
        min_angular_speed : float = 0.01 * self.max_angular_speed # min angular speed = 1% of max speed. #todo: make this a parameter (?)
        if abs(angle_error) < self.slowdown_angle:
            
            angular_speed = self.max_angular_speed * min((abs(angle_error) / self.slowdown_angle), 1.0)

        angular_speed = min(self.max_angular_speed, max(min_angular_speed, angular_speed))
        angular_speed *= rotation_direction

        angular_speed = max(angular_speed, self.lower_rotation_limit)
        angular_speed = min(angular_speed, self.upper_rotation_limit)

        twist_msg.angular.z = angular_speed
        self.cmd_publisher.publish(twist_msg)
        return False



    def follow_trajectory(self) -> bool:
        if self.robot_pose is None or self.target_waypoint is None:
            #rospy.logwarn(f"[Follower {self.robot_id}] Waiting for initialization")
            return False
        
        target_waypoint : Waypoint | None = self.update_target_point()
        if target_waypoint is None:
            return False
        
        distance_to_target : float = self.get_distance(self.robot_pose, target_waypoint)
        
        if distance_to_target < self.goal_tolerance:
            rospy.loginfo(f"[Follower {self.robot_id}] Reached the Goal Position with a tolerance of {distance_to_target:.3f} m")
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.cmd_publisher.publish(twist_msg)
            return True

        angle_to_target : float = self.get_angle(self.robot_pose, self.target_waypoint)
        steering_angle : float = angle_to_target - self.robot_yaw
        if steering_angle > np.pi:
            steering_angle -= 2 * np.pi
        elif steering_angle < -np.pi:
            steering_angle += 2 * np.pi

        distance_to_target = self.get_distance(self.robot_pose, self.target_waypoint)
        linear_speed, angular_speed = self.control_speeds(distance_to_target, steering_angle)
        linear_speed : float = linear_speed * max(np.cos(steering_angle), 0)

        # Apply Robot Harware Limits
        linear_speed = min(self.max_linear_speed, max(-self.max_linear_speed, linear_speed))
        angular_speed = min(self.max_angular_speed, max(-self.max_angular_speed, angular_speed))

        # Apply Safety Limits
        linear_speed = max(linear_speed, self.lower_linear_limit)
        linear_speed = min(linear_speed, self.upper_linear_limit)
        angular_speed = min(angular_speed, self.upper_rotation_limit)
        angular_speed = max(angular_speed, self.lower_rotation_limit)

        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed
        self.cmd_publisher.publish(twist_msg)
        return False

            
if __name__ == '__main__':
    rospy.init_node("path_follower")
    planner_id = rospy.get_param('~robot_id', default=-1)
    path_follower : PathFollower = PathFollower(planner_id)