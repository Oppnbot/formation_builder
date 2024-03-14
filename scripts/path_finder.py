#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-


# Bugfixes:
# todo: start/stop positioning are not scaling
# todo: don't communicate via magic numbers, use publisher and subscribers


# Code Quality:
# todo: replace waypoints by ros messages
# todo: distribute code to multiple nodes

# Additional Features:
# todo: option for wider paths to fit robot width (+map scaling)
# todo: assign new priorities when there is no viable path
# todo: assign new priorities when a path took too long (?)
# todo: implement path following
# todo: distribute calculation to multiple robots for O{(n-1)!} instead of O{n!}

# Other:
# todo: skip visited nodes ?
# todo: test in different scenarios
# todo: adjust to the robots real dynamics (?)
# todo: improve overall computing performance
# todo: improve occupation time calculations with safety margin etc

from __future__ import annotations
import rospy
import numpy as np
import cv2
import time
import heapq
#from commons import TrajectoryData, Waypoint
from formation_builder.msg import Trajectory, PixelPos, Formation, GoalPose
from formation_builder.msg import Waypoint as WaypointMsg
from visualization import fb_visualizer
#from formation_builder.srv import transformation
from formation_builder.srv import TransformPixelToWorld, TransformPixelToWorldResponse, TransformWorldToPixel, TransformWorldToPixelResponse
from geometry_msgs.msg import Pose, Point

from visualization_msgs.msg import Marker, MarkerArray
import matplotlib.pyplot as plt

class Waypoint():
    def __init__(self, pixel_pos : tuple[int, int], occupied_from: float, occupied_until : float = float('inf'), world_pos : tuple[float, float]|None = None, previous_waypoint:Waypoint|None = None):
        """
        Each grid cell the robot passes counts as a Waypoint. Multiple Waypoints make up the robots path.

        This Class contains the same data as the Waypoint msg but is needed for recursive operations and also offers an override for comparision etc, needed to execute the planning algorithm.
        
        :params world_pos: the (x, y) position in world coordinates [m]; used to navigate the robot
        :params pixel_pos: the (x, y) position in pixel coordinates [px]; used to find a path
        :params occupied_from: time when waypoint first becomes occupied, making it unavailable for othe
        :params occupied_until: time when waypoint becomes free, making it available for other robots [s]
        """
        self.world_pos : tuple[float, float] | None = world_pos # the (x, y) position in world coordinates [m]; used to navigate the robot
        self.pixel_pos : tuple[int, int] = pixel_pos            # the (x, y) position in pixel coordinates [px]; used to find a path
        self.occupied_from: float = occupied_from               # time when waypoint first becomes occupied, making it unavailable for other robots [s]
        self.occupied_until: float = occupied_until              # time when waypoint becomes free, making it available for other robots [s]
        self.previous_waypoint : Waypoint|None = previous_waypoint
        self.is_waiting_point : bool = False

    def __eq__(self, __value: Waypoint) -> bool:
        return self.pixel_pos == __value.pixel_pos

    def __lt__(self, other : Waypoint):
        return self.occupied_from < other.occupied_from
    
    def convert_to_msg(self) -> WaypointMsg:
        waypoint_msg : WaypointMsg = WaypointMsg()
        waypoint_msg.occupied_from = self.occupied_from
        waypoint_msg.occupied_until = self.occupied_until
        waypoint_msg.is_waiting_point = self.is_waiting_point
        
        if self.world_pos is not None:
            world_pos : Pose = Pose()
            world_pos.position.x = self.world_pos[0]
            world_pos.position.y = self.world_pos[1]
            waypoint_msg.world_position = world_pos

        pixel_pos : PixelPos = PixelPos()
        pixel_pos.x, pixel_pos.y = self.pixel_pos

        waypoint_msg.pixel_position = pixel_pos
        return waypoint_msg



class PathFinder:
    def __init__(self, planner_id : int = 0):
        # -------- CONFIG START --------
        self.allow_straights : bool = True   # allows the following movements: 0°, 90°, 180°, 380° 
        self.allow_diagonals : bool = True   # allows the following movements: 45°, 135°, 225°, 315°
        self.allow_knight_moves: bool = False # allows the following movements: 26°, 63°, 116°, 153°, 206°, 243°, 296°, 333° (like a knight in chess)

        self.check_dynamic_obstacles : bool = True
        self.dynamic_visualization : bool = False # publishes timing map after every step, very expensive
        self.kernel_size : int = 3 #!kernel size -> defines the safety margins for dynamic and static obstacles; grid_size * kernel_size = robot_size
        self.speed : float = 0.5 #todo: config or something smarter. for now 1.5 seems to be fine
        # -------- CONFIG END --------
        
        self.id: int = planner_id
        self.robot_pose : Pose | None = None
        rospy.Subscriber(f'/mir{self.id}/mir_pose_simple', Pose, self.update_pose)
        #self.trajectory_publisher : rospy.Publisher = rospy.Publisher('formation_builder/trajectory', Trajectory, queue_size=10, latch=True)
        return None


    def update_pose(self, pose: Pose) -> None:
        self.robot_pose = pose
        return None


    def bloat_path(self, waypoints : list[Waypoint]) -> list[Waypoint]:
        bloated_path : list[Waypoint] = []
        occupied_positions : dict[tuple[int, int], list[Waypoint]] = {}
        for waypoint in waypoints:
            if waypoint.pixel_pos not in occupied_positions.keys():
                occupied_positions[waypoint.pixel_pos] = []
            occupied_positions[waypoint.pixel_pos].append(waypoint)
            bloat_width : int = (self.kernel_size - 1) // 2
            for x in range(waypoint.pixel_pos[0] - bloat_width, waypoint.pixel_pos[0] + bloat_width + 1):
                for y in range(waypoint.pixel_pos[1] - bloat_width, waypoint.pixel_pos[1] + bloat_width +1):
                    if (x, y) not in occupied_positions.keys():
                        occupied_positions[x, y] = []
                    new_waypoint : Waypoint = Waypoint((x,y), waypoint.occupied_from, waypoint.occupied_until)
                    occupied_positions[(x, y)].append(new_waypoint)
        for position, waypoints in occupied_positions.items():
            if not waypoints:
                continue
            min_occupied_from : float = float('inf')
            max_occupied_until : float = 0
            for waypoint in waypoints:
                if waypoint.occupied_from < min_occupied_from:
                    min_occupied_from = waypoint.occupied_from
                if waypoint.occupied_until > max_occupied_until:
                    max_occupied_until = waypoint.occupied_until
            if min_occupied_from == float('inf'):
                rospy.logwarn("min occupation time is infinite")
            if max_occupied_until == 0:
                rospy.logwarn("max occupation time is 0")
            new_waypoint : Waypoint = Waypoint(position, min_occupied_from, max_occupied_until)
            bloated_path.append(new_waypoint)
        return bloated_path



    def search_path(self, static_obstacles: np.ndarray, goal_pos: GoalPose, dynamic_obstacles: list[Trajectory] = []) -> Trajectory | None:
        if self.robot_pose is None:
            rospy.logwarn(f"[Planner {self.id}] failed to plan path, since robot position is unknown.")
            return None
        rospy.loginfo(f"[Planner {self.id}] Starting Trajectory Search")

        start_time = time.time()

        # Get current position and transform it to pixel space
        transform_world_to_pixel = rospy.ServiceProxy('/formation_builder/world_to_pixel', TransformWorldToPixel)

        w2p_response : TransformWorldToPixelResponse = transform_world_to_pixel([self.robot_pose.position.x], [self.robot_pose.position.y])
        if len(w2p_response.x_pixel) == 0 or len(w2p_response.y_pixel) == 0:
            rospy.logwarn(f"[Planner {self.id}] failed to plan path, since robot starting position couldn't be converted to pixel space.")
            return None
        robot_start_pixel_pos : tuple[int, int] = (w2p_response.x_pixel[0], w2p_response.y_pixel[0])
        start_waypoint : Waypoint = Waypoint(robot_start_pixel_pos, 0.0)

        # Get goal position and transform it to pixel space
        w2p_response : TransformWorldToPixelResponse = transform_world_to_pixel([goal_pos.goal.position.x], [goal_pos.goal.position.y])
        if len(w2p_response.x_pixel) == 0 or len(w2p_response.y_pixel) == 0:
            rospy.logwarn(f"[Planner {self.id}] failed to plan path, since its goal position couldn't be converted to pixel space.")
            return None
        robot_goal_pixel_pos : tuple[int, int] = (w2p_response.x_pixel[0], w2p_response.y_pixel[0])
        goal_waypoint : Waypoint = Waypoint(robot_goal_pixel_pos, float('inf'))

        occupied_positions : dict[tuple[float, float], list[Waypoint]] = {}

        
        bloating_time_start = time.time()
        for dyn_obst in dynamic_obstacles:
            dynamic_obstacle : Trajectory = dyn_obst

            if dynamic_obstacle.occupied_positions is None:
                continue

            waypoints : list[Waypoint] = [Waypoint((waypoint_msg.pixel_position.x, waypoint_msg.pixel_position.y), waypoint_msg.occupied_from, waypoint_msg.occupied_until) for waypoint_msg in dynamic_obstacle.occupied_positions]
            bloated_dynamic_obstacles : list[Waypoint] = self.bloat_path(waypoints)
            for waypoint in bloated_dynamic_obstacles:
                occupied_positions.setdefault(waypoint.pixel_pos, []).append(waypoint)
        bloating_time_done = time.time() 
        rospy.loginfo(f"[Planner {self.id}] bloated paths. this took {bloating_time_done - bloating_time_start:.6f}s")

        heap: list[tuple[float, Waypoint]] = [(0, start_waypoint)]

        
        # Dilate the obstacles by ~1/2 of the robots size to avoid collisions
        dilation_time_start = time.time()
        kernel = np.ones((self.kernel_size, self.kernel_size),np.uint8) #type: ignore
        bloated_static_obstacles : np.ndarray = cv2.erode(static_obstacles, kernel) #-> erosion of free space = dilation of obstacles
        dilation_time_done = time.time()
        rospy.loginfo(f"[Planner {self.id}] Dilated map. This took {dilation_time_done-dilation_time_start:.6f}s")

        rows: int = bloated_static_obstacles.shape[0]
        cols: int = bloated_static_obstacles.shape[1]
        
        timings: np.ndarray = np.full((rows, cols), -1.0)
        timings[robot_start_pixel_pos] = 0.0

        
        direct_neighbors: list[tuple[int, int]] = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        diagonal_neighbors: list[tuple[int, int]] = [(1, 1), (-1, 1), (1, -1), (-1, -1)]
        knight_neighbors: list[tuple[int, int]] = [(2, 1), (-2, 1), (2, -1), (-2, -1), (1, 2), (-1, 2), (1, -2), (-1, -2)]
        neighbors: list[tuple[int, int]] = []
        if self.allow_straights:
            neighbors += direct_neighbors
        if self.allow_diagonals:
            neighbors += diagonal_neighbors
        if self.allow_knight_moves:
            neighbors += knight_neighbors
        neighbor_costs : list[float] = [np.hypot(x, y) / self.speed for x, y in neighbors]

        loop_time_start = time.time()
        iterations : int = 0
        while heap:
            iterations += 1
            current_cost, current_waypoint = heapq.heappop(heap)
            if iterations % 10_000 == 0:
                rospy.loginfo(f"[Planner {self.id}] {iterations} iterations done!")
            if iterations > 500_000:
                rospy.logwarn(f"[Planner {self.id}] breaking because algorithm reached max iterations")
                break
            if current_waypoint == goal_waypoint:
                rospy.loginfo(f"[Planner {self.id}] Reached the goal after {iterations} iterations")
                goal_waypoint = current_waypoint
                break
            if self.dynamic_visualization:
                fb_visualizer.draw_timings(timings, bloated_static_obstacles, start_waypoint.pixel_pos, goal_waypoint.pixel_pos, dynamic_obstacles=dynamic_obstacles, sleep=None)

            # CHECK CURRENT POSITION FOR COLLISION
            # if another robot is blocking the path, the robot waits for the path to become free. this may lead to collision with other robots that try to drive through this waiting position
            # the proposed solution is to check for other robots and if a conflict is found, the waiting position will be changed to its parent position.
            if self.check_dynamic_obstacles and (current_waypoint.pixel_pos in occupied_positions.keys()):
                is_occupied : bool = False
                for waypoint in occupied_positions[current_waypoint.pixel_pos]:
                    if current_waypoint.occupied_from <= waypoint.occupied_from:
                        #rospy.logwarn(f"robot {self.id} would collide at position {current_waypoint.pixel_pos} after {current_cost}s while waiting. it is occupied between {waypoint.occupied_from}s -> {waypoint.occupied_until}s ")
                        is_occupied = True
                        if current_waypoint.previous_waypoint is not None:
                            #rospy.loginfo(f"will wait at {current_waypoint.previous_waypoint.pixel_pos} until {waypoint.occupied_until}")
                            if waypoint.occupied_until == float('inf'):
                                continue
                            timings[current_waypoint.pixel_pos[0], current_waypoint.pixel_pos[1]] = -1
                            heapq.heappush(heap, (waypoint.occupied_until, current_waypoint.previous_waypoint)) # we have to add a small number because of floating point issues
                        else:
                            rospy.logwarn(f"[Planner {self.id}] will collide at current position {current_waypoint.pixel_pos} but previous waypoint is none!")
                        break
                if is_occupied:
                    continue

            # GRAPH EXPANSION
            # look at the neighbors; expand if possible. The neighbor has to contain free space, meaning no static/dynamic obstacle
            # we also have to assign a cost depending on the time needed to reach this node. this is the parent cost + an additional driving cost
            for index, (x_neighbor, y_neighbor) in enumerate(neighbors):
                x, y = current_waypoint.pixel_pos[0] + x_neighbor, current_waypoint.pixel_pos[1] + y_neighbor
                if 0 <= x < rows and 0 <= y < cols and bloated_static_obstacles[x, y] != 0: # check for static obstacles / out of bounds
  
                    driving_cost = current_cost + neighbor_costs[index]
                    # DYNAMIC OBSTACLE CHECK
                    # if the neighbor node is currently occupied by another robot, wait at the current position. To do that we add the current position back into the heap
                    # but with an updated timing, that is equal to the time needed for the robot to free the position.
                    if self.check_dynamic_obstacles and (x, y) in occupied_positions.keys(): # check for dynamic obstacles
                        is_occupied : bool = False
                        for waypoint in occupied_positions[(x, y)]:
                            if waypoint.occupied_from <= driving_cost <= waypoint.occupied_until:
                                if waypoint.pixel_pos == goal_pos: # goalpos was found but occupied -> wait and stop searching
                                    heap = [(waypoint.occupied_until, current_waypoint)]
                                heapq.heappush(heap, (waypoint.occupied_until, current_waypoint))
                                is_occupied = True
                                break
                        if is_occupied:
                            continue

                    # EXPANSION
                    # cell contains free space -> add it to the heap with the calculated driving cost. since the heap works like a priority queue, it will automatically be sorted,
                    # so that we proceed with the earliest remaining node.
                    if driving_cost < timings[x, y] or timings[x, y] < 0:
                        timings[x, y] = driving_cost
                        new_waypoint : Waypoint = Waypoint((x,y), driving_cost, previous_waypoint=current_waypoint)
                        heapq.heappush(heap, (driving_cost, new_waypoint))
            if not heap:
                rospy.loginfo(f"[Planner {self.id}] stopping because heap queue is empty")
        
        rospy.loginfo(f"[Planner {self.id}] stopped after a total of {iterations} iterations")
        loop_end_time = time.time()
        rospy.loginfo(f"[Planner {self.id}] planned path! Djikstras main loop took {loop_end_time-loop_time_start:.6f}s")
        

        #* --- Reconstruct Path ---
        pathfind_start_time = time.time()
        waypoints : list[Waypoint] = []
        current_waypoint : Waypoint | None = goal_waypoint
        max_travel_time : float = max(neighbor_costs)
        while current_waypoint:
            if current_waypoint.previous_waypoint is not None:
                current_waypoint.previous_waypoint.occupied_until = (current_waypoint.occupied_from + 1)* 2.2 + 3.0 # todo: define different metrics here #*1.3+1.0
                # Waiting Point Detection; A Waypoint is considered to be a waiting point if the following waypoint wont be reached in the max_travel_time between two point * small_factor
                if current_waypoint.occupied_from - current_waypoint.previous_waypoint.occupied_from > max_travel_time * 1.2:
                    current_waypoint.previous_waypoint.is_waiting_point = True
            waypoints.append(current_waypoint)
            current_waypoint = current_waypoint.previous_waypoint
            if self.dynamic_visualization:
                fb_visualizer.draw_timings(timings, bloated_static_obstacles, start_waypoint.pixel_pos, goal_waypoint.pixel_pos, dynamic_obstacles=dynamic_obstacles, sleep=None)
        waypoints.reverse()

        bloated_waypoints : list[Waypoint] = self.bloat_path(waypoints)

        
        #plot_values = [waypoint.occupied_from for waypoint in waypoints]
        #first_derivative = np.gradient(plot_values)
        #second_derivative = np.gradient(first_derivative)
        #plt.title(f"planner {self.id}")
        #plt.plot(plot_values, marker='o', linestyle='-')
        #plt.plot(first_derivative, marker='o', linestyle='-', label='1. derivative')
        #plt.plot(second_derivative, marker='o', linestyle='-', label='2. derivative')
        #plt.show()

        pathfind_done_time = time.time()
        rospy.loginfo(f"[Planner {self.id}] Found a path. This took {pathfind_done_time-pathfind_start_time:.6f}s")
        



        rospy.loginfo(f"[Planner {self.id}] Shortest path consists of {len(waypoints)} nodes with a cost of {timings[robot_goal_pixel_pos[0], robot_goal_pixel_pos[1]]}")

        # Transform Path from Pixel-Space to World-Space for visualization and path following
        trafo_start_time = time.time()

        transform_pixel_to_world = rospy.ServiceProxy('/formation_builder/pixel_to_world', TransformPixelToWorld)
        pixel_positions_x : list[int] = [waypoint.pixel_pos[0] for waypoint in waypoints]
        pixel_positions_y : list[int] = [waypoint.pixel_pos[1] for waypoint in waypoints]
        response : TransformPixelToWorldResponse = transform_pixel_to_world(pixel_positions_x, pixel_positions_y)
        for index, waypoint in enumerate(waypoints):
            #rospy.logwarn(f"world_pos {response.x_world[index]} / {response.y_world[index]}")
            waypoint.world_pos = (response.x_world[index], response.y_world[index])

        pixel_positions_x : list[int] = [waypoint.pixel_pos[0] for waypoint in bloated_waypoints]
        pixel_positions_y : list[int] = [waypoint.pixel_pos[1] for waypoint in bloated_waypoints]
        response : TransformPixelToWorldResponse = transform_pixel_to_world(pixel_positions_x, pixel_positions_y)
        for index, waypoint in enumerate(bloated_waypoints):
            waypoint.world_pos = (response.x_world[index], response.y_world[index])
        

        trafo_end_time = time.time()
        rospy.loginfo(f"[Planner {self.id}] Transformed pixel data to world coordinates. This took {trafo_end_time-trafo_start_time:.6f}s")
        
        # Visualization
        #fb_visualizer.draw_timings(timings, bloated_static_obstacles, start_pos, goal_pos, trajectory_data.waypoints)
        
        trajectory : Trajectory = Trajectory()
        trajectory.planner_id = self.id
        trajectory.goal_waypoint = goal_waypoint.convert_to_msg()
        trajectory.start_waypoint = start_waypoint.convert_to_msg()
        path_msgs : list[WaypointMsg] = [waypoint.convert_to_msg() for waypoint in waypoints]
        occupied_positions_msgs : list[WaypointMsg] = [waypoint.convert_to_msg() for waypoint in bloated_waypoints]
        trajectory.path = path_msgs
        trajectory.occupied_positions = occupied_positions_msgs
        #self.trajectory_publisher.publish(trajectory)

        end_time = time.time()
        rospy.loginfo(f"[Planner {self.id}] Done! Took {end_time-start_time:.6f}s in total for this planner.")
        rospy.loginfo(f"[Planner {self.id}] - - - - - - - - -")
        return trajectory