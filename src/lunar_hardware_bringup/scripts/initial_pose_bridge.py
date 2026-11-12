#!/usr/bin/python3
"""Convert RViz's map->base_link estimate into the upstream map->body seed."""

import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from interface.srv import Relocalize
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def pose_matrix(pose):
    q = pose.orientation
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(values)
    if not np.isfinite(values).all() or norm < 1e-12:
        raise ValueError('Invalid orientation quaternion')
    x, y, z, w = values / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    if not np.isfinite(matrix).all():
        raise ValueError('Invalid position')
    return matrix


def make_request(base_pose, base_from_body, pcd_path, base_z):
    """T_map_body = T_map_base * T_base_body; upstream uses Rz*Rx*Ry."""
    map_from_base = pose_matrix(base_pose)
    map_from_base[2, 3] = base_z
    map_from_body = map_from_base @ pose_matrix(base_from_body)
    rotation = map_from_body[:3, :3]
    roll = math.asin(float(np.clip(rotation[2, 1], -1.0, 1.0)))
    if abs(math.cos(roll)) > 1e-7:
        yaw = math.atan2(-rotation[0, 1], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], rotation[2, 2])
    else:
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        pitch = 0.0
    request = Relocalize.Request()
    request.pcd_path = pcd_path
    request.x, request.y, request.z = map(float, map_from_body[:3, 3])
    request.yaw, request.roll, request.pitch = yaw, roll, pitch
    return request


class InitialPoseBridge(Node):
    def __init__(self):
        super().__init__('initial_pose_bridge')
        self.declare_parameter('pcd_path', '')
        self.declare_parameter('base_z', 0.18108)
        self.pcd_path = str(Path(self.get_parameter('pcd_path').value).expanduser().resolve())
        if not Path(self.pcd_path).is_file():
            raise ValueError(f'PCD map does not exist: {self.pcd_path}')
        self.base_z = float(self.get_parameter('base_z').value)
        if not math.isfinite(self.base_z):
            raise ValueError('base_z must be finite')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.client = self.create_client(Relocalize, '/localizer/relocalize')
        self.pending = None
        self.inflight = None
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.receive_pose, 10)
        self.timer = self.create_timer(0.2, self.send_pending)
        self.get_logger().info('Ready: use 2D Pose Estimate for base_link in map. '
                               'No relocalization request is sent until you choose a pose.')

    def receive_pose(self, message):
        if message.header.frame_id != 'map':
            self.get_logger().error('Initial pose must be in map; set RViz Fixed Frame to map.')
            return
        self.pending = message.pose.pose

    def send_pending(self):
        if self.pending is None or self.inflight is not None:
            return
        if not self.client.service_is_ready():
            return
        try:
            # Static installation is read from URDF, including the configured mounting yaw.
            transform = self.buffer.lookup_transform('base_link', 'body', Time()).transform
            mount = Pose()
            mount.position.x = transform.translation.x
            mount.position.y = transform.translation.y
            mount.position.z = transform.translation.z
            mount.orientation = transform.rotation
            request = make_request(self.pending, mount, self.pcd_path, self.base_z)
        except TransformException:
            return  # Keep the latest click until robot_state_publisher has supplied TF.
        except ValueError as error:
            self.get_logger().error(str(error))
            self.pending = None
            return
        self.pending = None
        self.inflight = self.client.call_async(request)
        self.inflight.add_done_callback(self.request_done)
        self.get_logger().info(
            f'body seed: xyz=({request.x:.3f}, {request.y:.3f}, {request.z:.3f}), '
            f'yaw={request.yaw:.3f} rad')

    def request_done(self, future):
        self.inflight = None
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Map and initial pose accepted; ICP is matching. '
                                       'Use /localizer/relocalize_check to check initial match.')
            else:
                self.get_logger().error(f'Relocalization request failed: {response.message}')
        except Exception as error:
            self.get_logger().error(f'Relocalization service failed: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = InitialPoseBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
