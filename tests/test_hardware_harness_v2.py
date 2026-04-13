import sys
import types
import importlib.util
import unittest
import tempfile
import os
import csv
import math


def inject_fake_ros():
    # Minimal fake rclpy + message types so the module can be imported for unit tests
    rclpy_mod = types.ModuleType("rclpy")
    rclpy_node_mod = types.ModuleType("rclpy.node")

    class FakeNode:
        def __init__(self, *args, **kwargs):
            pass

        def create_publisher(self, *a, **k):
            class Pub:
                def publish(self, *a, **k):
                    pass

                def get_subscription_count(self):
                    return 1

            return Pub()

        def create_timer(self, *a, **k):
            return None

        def create_subscription(self, *a, **k):
            return None

        def get_logger(self):
            return types.SimpleNamespace(info=lambda *a, **k: None, error=lambda *a, **k: None)

        def get_clock(self):
            return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=0, to_msg=lambda: None))

    rclpy_node_mod.Node = FakeNode
    rclpy_mod.node = rclpy_node_mod
    # minimal QoS module needed by the harness import
    rclpy_qos = types.ModuleType("rclpy.qos")

    class QoSProfile:
        def __init__(self, depth=10, reliability=None):
            self.depth = depth
            self.reliability = reliability

    class ReliabilityPolicy:
        BEST_EFFORT = 0
        RELIABLE = 1

    rclpy_qos.QoSProfile = QoSProfile
    rclpy_qos.ReliabilityPolicy = ReliabilityPolicy

    rclpy_mod.qos = rclpy_qos
    sys.modules["rclpy"] = rclpy_mod
    sys.modules["rclpy.node"] = rclpy_node_mod
    sys.modules["rclpy.qos"] = rclpy_qos

    # geometry_msgs.msg
    geom = types.ModuleType("geometry_msgs")
    geom_msg = types.ModuleType("geometry_msgs.msg")

    class Pose:
        def __init__(self):
            self.position = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.orientation = types.SimpleNamespace(w=1.0)

    class PoseStamped:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None, frame_id="")
            self.pose = Pose()

    class Point:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    class Vector3:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    class WrenchStamped:
        def __init__(self):
            self.wrench = types.SimpleNamespace(
                force=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                torque=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            )

    geom_msg.Pose = Pose
    geom_msg.PoseStamped = PoseStamped
    geom_msg.Point = Point
    geom_msg.Vector3 = Vector3
    geom_msg.WrenchStamped = WrenchStamped
    geom.msg = geom_msg
    sys.modules["geometry_msgs"] = geom
    sys.modules["geometry_msgs.msg"] = geom_msg

    # sensor_msgs.msg
    sensor = types.ModuleType("sensor_msgs")
    sensor_msg = types.ModuleType("sensor_msgs.msg")

    class JointState:
        def __init__(self):
            self.velocity = []

    sensor_msg.JointState = JointState
    sensor.msg = sensor_msg
    sys.modules["sensor_msgs"] = sensor
    sys.modules["sensor_msgs.msg"] = sensor_msg

    # std_msgs.msg
    std = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")

    class Float64MultiArray:
        def __init__(self, data=None):
            self.data = data or []

    class Bool:
        def __init__(self, data=False):
            self.data = data

    std_msg.Float64MultiArray = Float64MultiArray
    std_msg.Bool = Bool
    std.msg = std_msg
    sys.modules["std_msgs"] = std
    sys.modules["std_msgs.msg"] = std_msg


class TestHardwareHarnessV2(unittest.TestCase):
    def setUp(self):
        inject_fake_ros()

        # import the module by path so tests work regardless of package layout
        module_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "tools", "hardware_harness_v2.py"))
        spec = importlib.util.spec_from_file_location("hardware_harness_v2", module_path)
        self.hh = importlib.util.module_from_spec(spec)
        sys.modules["hardware_harness_v2"] = self.hh
        spec.loader.exec_module(self.hh)

    def test_csv_write_helpers(self):
        harness = self.hh.HardwareHarnessAdaptive()
        tmpdir = tempfile.mkdtemp()
        path = os.path.join(tmpdir, "test.csv")
        cols = ["a", "b"]
        harness.write_csv_header(path, cols)
        with open(path, "r", newline="") as f:
            reader = csv.reader(f)
            header = next(reader)
        self.assertEqual(header, cols)

        row = {"a": 1, "b": 2}
        harness.append_csv_row(path, cols, row)
        with open(path, "r", newline="") as f:
            rows = list(csv.reader(f))
        self.assertEqual(rows[-1], ["1", "2"])

    def test_compute_press_target(self):
        harness = self.hh.HardwareHarnessAdaptive()
        harness.des1 = self.hh.Pose()
        harness.des2 = self.hh.Pose()

        target = harness.compute_press_target(-0.004, -0.006)
        self.assertIsNotNone(target)
        self.assertAlmostEqual(target["offset_x1"], -0.004)
        self.assertAlmostEqual(target["offset_x2"], -0.006)
        self.assertAlmostEqual(target["offset_y1"], 0.0)
        self.assertAlmostEqual(target["offset_z2"], 0.0)

    def test_contact_detected_threshold(self):
        harness = self.hh.HardwareHarnessAdaptive()
        harness.contact_valid_1 = True
        harness.contact_valid_2 = True
        harness.precontact_baseline_fx_1 = 0.20
        harness.precontact_baseline_fx_2 = 0.70
        harness.contact_fx_mag_1 = 0.75
        harness.contact_fx_mag_2 = 0.79

        self.assertAlmostEqual(harness.contact_threshold(1), 0.70)
        self.assertAlmostEqual(harness.contact_threshold(2), 0.80)
        self.assertTrue(harness.contact_detected(1))
        self.assertFalse(harness.contact_detected(2))

    def test_columns_defined(self):
        harness = self.hh.HardwareHarnessAdaptive()
        self.assertIn("timestamp", harness.log_columns)
        self.assertIn("desired_x_diff", harness.log_columns)
        self.assertIn("press_offset_x1", harness.log_columns)
        self.assertIn("contact_threshold_1", harness.log_columns)
        self.assertIn("timestamp", harness.sync_columns)


if __name__ == "__main__":
    unittest.main()
