#!/usr/bin/env python3
"""Save map from Hector SLAM to PGM/YAML files."""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image
import yaml

class MapSaver(Node):
    def __init__(self, map_name):
        super().__init__('map_saver')
        self.map_name = map_name
        self.saved = False

        # Match Hector SLAM QoS (volatile)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos
        )
        self.get_logger().info('Waiting for map...')

    def map_callback(self, msg):
        if self.saved:
            return

        self.get_logger().info(f'Received map: {msg.info.width}x{msg.info.height}')

        # Convert to image
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))

        # OccupancyGrid: -1=unknown, 0=free, 100=occupied
        # PGM: 205=unknown, 254=free, 0=occupied
        img_data = np.zeros((height, width), dtype=np.uint8)
        img_data[data == -1] = 205  # unknown
        img_data[data == 0] = 254   # free
        img_data[data == 100] = 0   # occupied
        # Scale intermediate values
        mask = (data > 0) & (data < 100)
        img_data[mask] = (100 - data[mask]) * 254 // 100

        # Flip vertically (ROS maps have origin at bottom-left)
        img_data = np.flipud(img_data)

        # Save PGM
        img = Image.fromarray(img_data, mode='L')
        pgm_path = f'{self.map_name}.pgm'
        img.save(pgm_path)
        self.get_logger().info(f'Saved {pgm_path}')

        # Save YAML
        yaml_data = {
            'image': f'{self.map_name.split("/")[-1]}.pgm',
            'resolution': msg.info.resolution,
            'origin': [msg.info.origin.position.x, msg.info.origin.position.y, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }
        yaml_path = f'{self.map_name}.yaml'
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)
        self.get_logger().info(f'Saved {yaml_path}')

        self.saved = True


def main():
    if len(sys.argv) < 2:
        print('Usage: save_map.py <map_name>')
        print('Example: save_map.py /home/owner/hector_ws/maps/office')
        sys.exit(1)

    map_name = sys.argv[1]

    rclpy.init()
    node = MapSaver(map_name)

    while rclpy.ok() and not node.saved:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()
    print('Map saved successfully!')


if __name__ == '__main__':
    main()
