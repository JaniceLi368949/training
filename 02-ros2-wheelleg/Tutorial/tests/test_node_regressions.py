"""Node logic tests with real MuJoCo physics and lightweight ROS API doubles.

These tests do not exercise DDS, executors, or generated ROS message bindings.
"""
import importlib.util
import math
import os
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import mujoco
import mujoco.viewer


SRC = Path(__file__).resolve().parents[1] / 'ros2_ws' / 'src'


def vector():
    return SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0)


def imu_message():
    return SimpleNamespace(header=SimpleNamespace(), orientation=vector(),
                           angular_velocity=vector())


def joint_message():
    return SimpleNamespace(header=SimpleNamespace(), name=[], position=[], velocity=[])


def transform_message():
    return SimpleNamespace(header=SimpleNamespace(),
                           transform=SimpleNamespace(translation=vector(), rotation=vector()))


class FakeNode:
    overrides = {}

    def __init__(self, name):
        self.parameters = {}
        self.logger = Mock()
        self.clock = Mock()
        self.clock.now.return_value.nanoseconds = 0
        self.timers = []

    def declare_parameter(self, name, default):
        self.parameters[name] = self.overrides.get(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, *args):
        return Mock()

    def create_subscription(self, *args):
        return Mock()

    def create_timer(self, period, callback):
        self.timers.append((period, callback))
        return Mock()

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return self.clock


def load_node(package, module, class_name):
    """Import the production module, replacing only unavailable ROS interfaces."""
    attributes = {
        'rclpy': {},
        'rclpy.node': {'Node': FakeNode},
        'ament_index_python': {},
        'ament_index_python.packages': {
            'get_package_share_directory': lambda name: str(SRC / name)},
        'builtin_interfaces': {},
        'builtin_interfaces.msg': {'Time': SimpleNamespace},
        'rm_msgs': {},
        'rm_msgs.msg': {'WheelLegCmd': SimpleNamespace},
        'sensor_msgs': {},
        'sensor_msgs.msg': {'Imu': imu_message, 'JointState': joint_message},
        'geometry_msgs': {},
        'geometry_msgs.msg': {'Twist': SimpleNamespace, 'TransformStamped': transform_message},
        'rosgraph_msgs': {},
        'rosgraph_msgs.msg': {'Clock': SimpleNamespace},
        'tf2_ros': {'StaticTransformBroadcaster': lambda node: Mock(),
                    'TransformBroadcaster': lambda node: Mock()},
    }
    modules = {}
    for name, attrs in attributes.items():
        modules[name] = ModuleType(name)
        modules[name].__dict__.update(attrs)
    spec = importlib.util.spec_from_file_location(
        '_test_' + module, SRC / package / package / (module + '.py'))
    assert spec is not None and spec.loader is not None
    loaded = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(loaded)
    return getattr(loaded, class_name)


class NodeRegressions(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.Sim = load_node('wheel_leg_sim', 'wheel_leg_sim', 'WheelLegSim')
        cls.Bridge = load_node('wheel_leg_bridge', 'wheel_leg_bridge', 'WheelLegBridge')
        cls.TF = load_node('rm_tf_demo', 'tf_broadcaster', 'TFBroadcaster')

    def make_headless_sim(self):
        with patch.object(FakeNode, 'overrides', {'viewer': False}):
            return self.Sim()

    def assert_valid_initial_state(self, node):
        imu = node.imu_pub.publish.call_args.args[0]
        self.assertEqual((imu.orientation.x, imu.orientation.y,
                          imu.orientation.z, imu.orientation.w), (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(node.data.time, 0.0)
        self.assertEqual(node.steps, 0)
        js = node.joint_pub.publish.call_args.args[0]
        self.assertEqual(len(js.name), 6)
        self.assertEqual(len(js.position), 6)
        self.assertEqual(len(js.velocity), 6)

    def test_headless_initial_state_is_valid_without_stepping(self):
        self.assert_valid_initial_state(self.make_headless_sim())

    def test_heartbeat_republishes_valid_state_without_stepping(self):
        node = self.make_headless_sim()
        node._last_cmd_mono -= 1.0
        node.heartbeat()
        self.assertEqual(node.imu_pub.publish.call_count, 2)
        self.assert_valid_initial_state(node)

    def test_one_command_still_advances_exactly_one_timestep(self):
        node = self.make_headless_sim()
        node.on_cmd(SimpleNamespace(right_phi1=0.1, right_phi4=-0.2,
                                    left_phi1=0.3, left_phi4=-0.4,
                                    right_wheel=-0.5, left_wheel=0.6))
        self.assertAlmostEqual(node.data.time, node.dt)
        self.assertEqual(node.steps, 1)
        self.assertEqual(list(node.data.ctrl), [0.1, -0.2, 0.3, -0.4, -0.5, 0.6])

    def test_linux_without_display_skips_viewer_before_launch(self):
        with patch('sys.platform', 'linux'), patch.dict(os.environ, {}, clear=True), \
                patch('mujoco.viewer.launch_passive') as launch:
            node = self.Sim()
        launch.assert_not_called()
        self.assertIsNone(node.viewer)
        self.assert_valid_initial_state(node)

    def test_display_environment_still_launches_requested_viewer(self):
        for display in ('DISPLAY', 'WAYLAND_DISPLAY'):
            with self.subTest(display=display), patch('sys.platform', 'linux'), \
                    patch.dict(os.environ, {display: ':test'}, clear=True), \
                    patch('mujoco.viewer.launch_passive') as launch:
                node = self.Sim()
                launch.assert_called_once_with(node.model, node.data)
                self.assertIsNotNone(node.viewer)

    def test_forward_and_reverse_compensate_mirrored_wheel_axes(self):
        node = self.Bridge()
        mujoco.mj_forward(node.model, node.data)
        self.assertAlmostEqual(node.data.xaxis[node.model.joint('right_wheel_torque').id][1], -1.0)
        self.assertAlmostEqual(node.data.xaxis[node.model.joint('left_wheel_torque').id][1], 1.0)
        for speed in (0.5, -0.5):
            with self.subTest(speed=speed):
                node.cmd_vel_cb(SimpleNamespace(linear=SimpleNamespace(x=speed),
                                                angular=SimpleNamespace(z=0.0)))
                node.step()
                self.assertAlmostEqual(node.data.ctrl[4], -speed)
                self.assertAlmostEqual(node.data.ctrl[5], speed)

    def test_pure_yaw_has_opposite_rolling_directions(self):
        node = self.Bridge()
        for yaw in (0.3, -0.3):
            with self.subTest(yaw=yaw):
                node.cmd_vel_cb(SimpleNamespace(linear=SimpleNamespace(x=0.0),
                                                angular=SimpleNamespace(z=yaw)))
                self.assertAlmostEqual(node.right_torque, -yaw * node.track / 2)
                self.assertAlmostEqual(node.left_torque, -yaw * node.track / 2)

    def test_camera_pitch_rotates_about_y_not_x(self):
        node = self.TF()
        for seconds in (math.pi / 2, 3 * math.pi / 2):
            with self.subTest(seconds=seconds):
                ns = int(seconds * 1e9)
                node.clock.now.return_value.nanoseconds = ns
                node.broadcast_camera()
                rotation = node.dynamic_br.sendTransform.call_args.args[0].transform.rotation
                pitch = 0.6 * math.sin(ns * 1e-9)
                self.assertEqual(rotation.x, 0.0)
                self.assertEqual(rotation.z, 0.0)
                self.assertAlmostEqual(rotation.y, math.sin(pitch / 2))
                self.assertAlmostEqual(rotation.w, math.cos(pitch / 2))


if __name__ == '__main__':
    unittest.main()
