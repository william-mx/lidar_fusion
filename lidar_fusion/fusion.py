#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix

from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import LaserScan

from message_filters import ApproximateTimeSynchronizer, Subscriber

from lidar_fusion.utils import read_camera_config
from ros2_numpy import from_detection2d_array, scan_to_np
import numpy as np


class SyncedFusionNode(Node):
    def __init__(self, config_path):
        super().__init__('synced_fusion_node')

        # Load camera calibration
        self.K, self.T = read_camera_config(config_path)
        self.get_logger().info(f'Loaded Intrinsic Matrix (K):\n{self.K}')
        self.get_logger().info(f'Loaded Extrinsic Matrix (T):\n{self.T}')

        # Sensor QoS
        qos = qos_profile_sensor_data
        qos.depth = 1

        # Subscribers using message_filters
        self.detection_sub = Subscriber(self, Detection2DArray, '/detections_2d', qos_profile=qos)
        self.scan_sub = Subscriber(self, LaserScan, '/scan', qos_profile=qos)

        self.ats = ApproximateTimeSynchronizer(
            [self.detection_sub, self.scan_sub],
            queue_size=10,
            slop=0.3
        )
        self.ats.registerCallback(self.fusion_callback)

        self.get_logger().info('Subscribed to /detections_2d and /scan using message_filters')

    def fusion_callback(self, detection_msg: Detection2DArray, scan_msg: LaserScan):
        # Timestamps
        detection_sec = detection_msg.header.stamp.sec + 1e-9 * detection_msg.header.stamp.nanosec
        scan_sec = scan_msg.header.stamp.sec + 1e-9 * scan_msg.header.stamp.nanosec
        dt = detection_sec - scan_sec

        # Parse detections
        detections, timestamp_2d = from_detection2d_array(detection_msg)

        # Convert scan to (x, y, intensity)
        xyi, timestamp_scan = scan_to_np(scan_msg)
        x, y, intensity = xyi.T

        # Create 3D points in vehicle coordinates (z = 0)
        self.pts = np.column_stack([x, y, np.zeros_like(x)])

        dt = timestamp_2d - timestamp_scan
        self.get_logger().info(f'Δt = {dt:.6f} s')

def main(args=None):
    rclpy.init(args=args)

    pkg_path = get_package_prefix('lidar_fusion').replace('install', 'src')
    config_path = f'{pkg_path}/config/calibration.yaml'

    node = SyncedFusionNode(config_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
