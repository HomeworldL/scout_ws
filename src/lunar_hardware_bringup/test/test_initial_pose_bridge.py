"""Regression checks for the vehicle/IMU reference and upstream Euler convention."""
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest
from geometry_msgs.msg import Pose

spec = importlib.util.spec_from_file_location(
    'initial_pose_bridge', Path(__file__).parents[1] / 'scripts/initial_pose_bridge.py')
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


def pose(x=0.0, y=0.0, z=0.0, yaw=0.0):
    result = Pose()
    result.position.x, result.position.y, result.position.z = x, y, z
    result.orientation.z, result.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
    return result


def upstream_rotation(request):
    a, b, c = request.yaw, request.roll, request.pitch
    rz = np.array([[math.cos(a), -math.sin(a), 0],
                   [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    rx = np.array([[1, 0, 0], [0, math.cos(b), -math.sin(b)],
                   [0, math.sin(b), math.cos(b)]])
    ry = np.array([[math.cos(c), 0, math.sin(c)], [0, 1, 0],
                   [-math.sin(c), 0, math.cos(c)]])
    return rz @ rx @ ry


@pytest.mark.parametrize('yaw', [0.0, math.pi/2, -math.pi/2, math.pi])
def test_car_heading_and_sensor_offset(yaw):
    # This is body in base_link, independently derived from the actual installation.
    mount = pose(0.17871, 0.011, 0.05738, math.pi/2)
    request = bridge.make_request(pose(3.0, 4.0, yaw=yaw), mount, '/map.pcd', 0.18108)
    expected = [3+math.cos(yaw)*0.17871-math.sin(yaw)*0.011,
                4+math.sin(yaw)*0.17871+math.cos(yaw)*0.011, 0.23846]
    np.testing.assert_allclose([request.x, request.y, request.z], expected, atol=1e-9)
    expected_rotation = bridge.pose_matrix(pose(yaw=yaw+math.pi/2))[:3, :3]
    np.testing.assert_allclose(upstream_rotation(request), expected_rotation, atol=1e-9)
    assert request.pcd_path == '/map.pcd'


@pytest.mark.parametrize('roll', [0.2, -0.4, math.pi/2, -math.pi/2])
def test_full_rotation_and_upstream_zxy_order(roll):
    # A non-planar mount catches accidental use of the conventional Rz*Ry*Rx order.
    mount = pose(0.1, -0.2, 0.3)
    mount.orientation.x, mount.orientation.w = math.sin(roll/2), math.cos(roll/2)
    base = pose(1.0, 2.0, yaw=0.7)
    request = bridge.make_request(base, mount, '/map.pcd', 0.9)
    expected = bridge.pose_matrix(base) @ bridge.pose_matrix(mount)
    np.testing.assert_allclose(upstream_rotation(request), expected[:3, :3], atol=1e-7)
    assert request.z == pytest.approx(1.2)


def test_nonzero_pitch_uses_upstream_order():
    mount = pose()
    mount.orientation.x = 0.1
    mount.orientation.y = 0.2
    mount.orientation.z = 0.3
    mount.orientation.w = 0.9
    base = pose(yaw=-1.2)
    request = bridge.make_request(base, mount, '/map.pcd', 0.18)
    expected = bridge.pose_matrix(base) @ bridge.pose_matrix(mount)
    np.testing.assert_allclose(upstream_rotation(request), expected[:3, :3], atol=1e-9)


@pytest.mark.parametrize('value', [float('nan'), float('inf')])
def test_reject_invalid_position(value):
    with pytest.raises(ValueError):
        bridge.make_request(pose(x=value), pose(), '/map.pcd', 0.18)


def test_reject_zero_quaternion():
    invalid = Pose()
    invalid.orientation.w = 0.0
    with pytest.raises(ValueError):
        bridge.make_request(invalid, pose(), '/map.pcd', 0.18)
