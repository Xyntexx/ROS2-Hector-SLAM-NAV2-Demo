#!/usr/bin/env python3
"""
Frontier-based exploration script for the custom 4-wheel robot.
The robot analyzes the occupancy grid map to find unexplored frontiers,
selects reachable targets, and uses Nav2 to navigate to them.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
from scipy.ndimage import binary_dilation
import random


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )

        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # State variables
        self.current_map = None
        self.exploring = False
        self.goal_handle = None

        # Parameters
        self.frontier_threshold = 10  # Number of unknown cells to be a frontier
        self.exploration_radius = 2.0  # meters - minimum distance between targets

        # Timer for exploration cycle
        self.timer = self.create_timer(5.0, self.exploration_cycle)

        self.get_logger().info('Frontier Explorer started! Waiting for map...')

    def map_callback(self, msg):
        """Process incoming map data."""
        self.current_map = msg

    def find_frontiers(self, grid_data, width, height):
        """
        Find frontier cells - boundaries between free space and unknown space.
        Returns list of (x, y) coordinates in grid frame.
        """
        grid = np.array(grid_data).reshape((height, width))

        # Define cell types
        FREE = (grid >= 0) & (grid < 50)
        UNKNOWN = grid == -1
        OCCUPIED = grid >= 50

        # Find free cells adjacent to unknown cells
        unknown_dilated = binary_dilation(UNKNOWN, iterations=1)
        frontiers = FREE & unknown_dilated

        # Get frontier coordinates
        frontier_coords = np.argwhere(frontiers)

        return frontier_coords

    def cluster_frontiers(self, frontier_coords, min_size=5):
        """
        Group nearby frontier cells into clusters.
        Returns centroids of clusters.
        """
        if len(frontier_coords) == 0:
            return []

        # Simple clustering by proximity
        clusters = []
        visited = set()

        for i, coord in enumerate(frontier_coords):
            if i in visited:
                continue

            cluster = [coord]
            visited.add(i)

            # Find nearby points
            for j, other_coord in enumerate(frontier_coords):
                if j not in visited:
                    dist = np.linalg.norm(coord - other_coord)
                    if dist < 10:  # Grid cells
                        cluster.append(other_coord)
                        visited.add(j)

            if len(cluster) >= min_size:
                centroid = np.mean(cluster, axis=0)
                clusters.append(centroid)

        return clusters

    def grid_to_world(self, grid_x, grid_y, map_msg):
        """Convert grid coordinates to world coordinates."""
        world_x = map_msg.info.origin.position.x + (grid_y + 0.5) * map_msg.info.resolution
        world_y = map_msg.info.origin.position.y + (grid_x + 0.5) * map_msg.info.resolution
        return world_x, world_y

    def select_best_frontier(self, centroids, map_msg):
        """
        Select the best frontier to explore based on distance and size.
        Returns world coordinates (x, y) or None.
        """
        if not centroids:
            return None

        # For simplicity, pick a random frontier
        # In a more sophisticated version, you'd use distance to robot
        selected = random.choice(centroids)

        world_x, world_y = self.grid_to_world(selected[0], selected[1], map_msg)
        return world_x, world_y

    def send_navigation_goal(self, x, y):
        """Send a navigation goal to Nav2."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Sending goal to Nav2: ({x:.2f}, {y:.2f})')

        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response from Nav2."""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected by Nav2')
            self.exploring = False
            return

        self.get_logger().info('Goal accepted by Nav2')
        self.exploring = True

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle navigation result."""
        result = future.result().result
        self.exploring = False
        self.get_logger().info('Navigation completed')

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        pass

    def exploration_cycle(self):
        """Main exploration loop - find and navigate to frontiers."""
        if self.exploring:
            self.get_logger().info('Already exploring, waiting...')
            return

        if self.current_map is None:
            self.get_logger().info('No map received yet')
            return

        self.get_logger().info('Searching for frontiers...')

        # Find frontiers
        frontier_coords = self.find_frontiers(
            self.current_map.data,
            self.current_map.info.width,
            self.current_map.info.height
        )

        if len(frontier_coords) == 0:
            self.get_logger().info('No frontiers found - exploration complete!')
            return

        self.get_logger().info(f'Found {len(frontier_coords)} frontier cells')

        # Cluster frontiers
        centroids = self.cluster_frontiers(frontier_coords)

        if not centroids:
            self.get_logger().info('No significant frontier clusters found')
            return

        self.get_logger().info(f'Found {len(centroids)} frontier clusters')

        # Select best frontier
        target = self.select_best_frontier(centroids, self.current_map)

        if target:
            self.send_navigation_goal(target[0], target[1])
        else:
            self.get_logger().info('Could not select a frontier target')


def main(args=None):
    rclpy.init(args=args)
    explorer = FrontierExplorer()

    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
