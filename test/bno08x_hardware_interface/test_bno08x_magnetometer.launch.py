"""
Launch integration test for bno08x_hardware_interface.

Launches the full ros2_control stack (robot_state_publisher, ros2_control_node,
imu_sensor_broadcaster and magnetometer_broadcaster) against the physical BNO08X
on /dev/i2c-1 at 0x28 when present, or in mock mode (enable_mock_mode:=true) when
the hardware is not detected, so the suite always runs in CI regardless of sensor availability.
"""

# TODO(rbscr) change 'bus' tests, use device

import math
import os
import subprocess
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy


# ── Hardware detection (evaluated once at import time) ────────────────────────

def _bno08x_available() -> bool:
    """Return True if BNO08X responds at /dev/i2c-1, address 0x4A."""
    # Deliberate leaving the fourth param of i2cget out.
    # The BNO08X sensor doesn't have a chip-ID to check.
    if not os.path.exists('/dev/i2c-1'):
        return False
    try:
        result = subprocess.run(
            ['i2cget', '-y', '1', '0x4A' ],
            capture_output=True,
            timeout=3,
        )
        return result.returncode == 0
    except Exception:
        return False


BNO08X_AVAILABLE = _bno08x_available()


# ── Launch description ────────────────────────────────────────────────────────

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """
    Launch the bno08x stack against real or mock hardware.

    When the BNO08X is absent the stack is launched with enable_mock_mode:=true
    so all tests run against the software stub rather than being skipped.
    """
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from ament_index_python.packages import get_package_share_directory

    pkg_share = get_package_share_directory('bno08x_hardware_interface')
    bno08x_launch = os.path.join(pkg_share, 'launch', 'bno08x_magnetometer.launch.py')

    launch_args = {
        'i2c_bus':                '1',
        'i2c_addr':               '4A',
        'axis_remap':             'East-North-Up',
        'imu_rate':               '100',
        'enable_magnetometer':    'true',
        'magnetometer_rate':      '100',
        'broadcast_magnetometer': 'true',
        'publish_diagnostics':    'false',
    }
    # TODO(rbscr)  diagnostics temporarily disabled, will be future enhancement

    if not BNO08X_AVAILABLE:
        launch_args['enable_mock_mode'] = 'true'

    bno08x = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bno08x_launch),
        launch_arguments=launch_args.items(),
    )

    return launch.LaunchDescription([
        bno08x,
        launch_testing.actions.ReadyToTest(),
    ])


# ── Test helpers ──────────────────────────────────────────────────────────────

def _wait_for_topic(node, topic, msg_type, timeout_sec=45.0):
    """Spin until at least one message is received on *topic*, then return the list."""
    received = []
    sub = node.create_subscription(msg_type, topic, received.append, 10)
    deadline = time.time() + timeout_sec
    while time.time() < deadline and not received:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return received


def _wait_for_controller_state(node, controller_name, timeout_sec=45.0):
    """Poll list_controllers until *controller_name* reaches 'active'; return its state."""
    from controller_manager_msgs.srv import ListControllers

    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    if not client.wait_for_service(timeout_sec=timeout_sec):
        node.destroy_client(client)
        return None

    state = None
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        future = client.call_async(ListControllers.Request())
        poll = time.time() + 5.0
        while time.time() < poll and not future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        if future.done():
            for c in future.result().controller:
                if c.name == controller_name:
                    state = c.state
            if state == 'active':
                break
        time.sleep(0.5)

    node.destroy_client(client)
    return state


# ── Test class ────────────────────────────────────────────────────────────────

class TestBNO08XLaunch(unittest.TestCase):
    """
    Integration tests for the full BNO08X ros2_control stack.

    Uses mock mode when hardware is absent.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_bno08x_launch_node')

    def tearDown(self):
        if hasattr(self, 'node'):
            self.node.destroy_node()

    # ── Controller manager ────────────────────────────────────────────────

    def test_controller_manager_service_available(self):
        """Verify /controller_manager/list_controllers service is reachable."""
        from controller_manager_msgs.srv import ListControllers
        client = self.node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.assertTrue(
            client.wait_for_service(timeout_sec=45.0),
            '/controller_manager/list_controllers not available within 45 s',
        )
        self.node.destroy_client(client)

    def test_imu_broadcaster_is_active(self):
        """Verify imu_sensor_broadcaster reaches 'active' state."""
        state = _wait_for_controller_state(self.node, 'imu_sensor_broadcaster')
        self.assertIsNotNone(
            state, 'imu_sensor_broadcaster not found in controller list')
        self.assertEqual(
            state, 'active',
            f'imu_sensor_broadcaster state is "{state}", expected "active"',
        )

    def test_magnetometer_broadcaster_is_active(self):
        """Verify magnetometer_broadcaster reaches 'active' state."""
        state = _wait_for_controller_state(self.node, 'magnetometer_broadcaster')
        self.assertIsNotNone(
            state, 'magnetometer_broadcaster not found in controller list')
        self.assertEqual(
            state, 'active',
            f'magnetometer_broadcaster state is "{state}", expected "active"',
        )

    # ── IMU topic ─────────────────────────────────────────────────────────
    # Note: The tests for the IMU topic could be removed.
    #       These are already done in the other launch-test 'test_bno08x.launch.py'

    def test_imu_topic_published(self):
        """Verify /imu_sensor_broadcaster/imu publishes at least one message."""
        from sensor_msgs.msg import Imu
        msgs = _wait_for_topic(
            self.node, '/imu_sensor_broadcaster/imu', Imu, timeout_sec=45.0)
        self.assertTrue(msgs, '/imu_sensor_broadcaster/imu not received within 45 s')

    def test_imu_frame_id(self):
        """Verify Imu messages carry frame_id = 'imu_frame'."""
        from sensor_msgs.msg import Imu
        msgs = _wait_for_topic(
            self.node, '/imu_sensor_broadcaster/imu', Imu, timeout_sec=45.0)
        self.assertTrue(msgs, '/imu_sensor_broadcaster/imu not received')
        self.assertEqual(
            msgs[0].header.frame_id, 'imu_frame',
            f'Expected frame_id "imu_frame", got "{msgs[0].header.frame_id}"',
        )

    def test_quaternion_norm_is_unity(self):
        """Verify the published quaternion represents a valid rotation (norm ≈ 1)."""
        from sensor_msgs.msg import Imu
        msgs = _wait_for_topic(
            self.node, '/imu_sensor_broadcaster/imu', Imu, timeout_sec=45.0)
        self.assertTrue(msgs, '/imu_sensor_broadcaster/imu not received')

        q = msgs[0].orientation
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        self.assertAlmostEqual(
            norm, 1.0, delta=0.05,
            msg=f'Quaternion norm {norm:.4f} is not close to 1.0',
        )

    def test_angular_velocity_is_finite(self):
        """Verify all angular velocity components are finite numbers."""
        from sensor_msgs.msg import Imu
        msgs = _wait_for_topic(
            self.node, '/imu_sensor_broadcaster/imu', Imu, timeout_sec=45.0)
        self.assertTrue(msgs, '/imu_sensor_broadcaster/imu not received')

        av = msgs[0].angular_velocity
        for axis, val in (('x', av.x), ('y', av.y), ('z', av.z)):
            self.assertTrue(
                math.isfinite(val),
                f'angular_velocity.{axis} = {val} is not finite',
            )

    def test_linear_acceleration_is_finite(self):
        """Verify all linear acceleration components are finite numbers."""
        from sensor_msgs.msg import Imu
        msgs = _wait_for_topic(
            self.node, '/imu_sensor_broadcaster/imu', Imu, timeout_sec=45.0)
        self.assertTrue(msgs, '/imu_sensor_broadcaster/imu not received')

        la = msgs[0].linear_acceleration
        for axis, val in (('x', la.x), ('y', la.y), ('z', la.z)):
            self.assertTrue(
                math.isfinite(val),
                f'linear_acceleration.{axis} = {val} is not finite',
            )

    def test_imu_publishes_continuously(self):
        """Verify the IMU topic publishes multiple messages over 2 seconds."""
        from sensor_msgs.msg import Imu

        # First wait for the topic to be alive, then collect for 2 s
        msgs = _wait_for_topic(
            self.node, '/imu_sensor_broadcaster/imu', Imu, timeout_sec=45.0)
        self.assertTrue(msgs, '/imu_sensor_broadcaster/imu not received')

        # Collect for ~2 more seconds
        received = []
        sub = self.node.create_subscription(
            Imu, '/imu_sensor_broadcaster/imu', received.append, 50)
        deadline = time.time() + 2.0
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.node.destroy_subscription(sub)

        self.assertGreater(
            len(received), 1,
            f'IMU topic published only {len(received)} message(s) in 2 s'
            ' — expected continuous stream',
        )

    # ── Magnetic messages  ───────────────────────────────────────

    def test_magnetic_field_topic_published(self):
        """Verify /magnetometer_broadcaster/magnetic_field publishes at least one message."""
        from sensor_msgs.msg import MagneticField
        msgs = _wait_for_topic(
            self.node, '/magnetometer_broadcaster/magnetic_field', MagneticField, timeout_sec=45.0)
        self.assertTrue(msgs, '/magnetometer_broadcaster/magnetic_field not received within 45 s')

    def test_magnetometer_imu_frame_id(self):
        """Verify magnetic_field messages carry frame_id = 'imu_frame'."""
        from sensor_msgs.msg import MagneticField
        msgs = _wait_for_topic(
            self.node, '/magnetometer_broadcaster/magnetic_field', MagneticField, timeout_sec=45.0)
        self.assertTrue(msgs, '/magnetometer_broadcaster/magnetic_field not received')
        self.assertEqual(
            msgs[0].header.frame_id, 'imu_frame',
            f'Expected frame_id "imu_frame", got "{msgs[0].header.frame_id}"',
        )

    def test_magnetic_field_is_finite(self):
        """Verify all magnetic field components are finite numbers."""
        from sensor_msgs.msg import MagneticField
        msgs = _wait_for_topic(
            self.node, '/magnetometer_broadcaster/magnetic_field', MagneticField, timeout_sec=45.0)
        self.assertTrue(msgs, '/magnetometer_broadcaster/magnetic_field not received')

        mf = msgs[0].magnetic_field
        for field, val in (('x', mf.x), ('y', mf.y), ('z', mf.z)):
            self.assertTrue(
                math.isfinite(val),
                f'magnetic_field.{field} = {val} is not finite',
            )

    def test_magnetic_field_publishes_continuously(self):
        """Verify the magnetic_field topic publishes multiple messages over 2 seconds."""
        from sensor_msgs.msg import MagneticField

        # First wait for the topic to be alive, then collect for 2 s
        msgs = _wait_for_topic(
            self.node, '/magnetometer_broadcaster/magnetic_field', MagneticField, timeout_sec=45.0)
        self.assertTrue(msgs, '/magnetometer_broadcaster/magnetic_field not received')

        # Collect for ~2 more seconds
        received = []
        sub = self.node.create_subscription(
            MagneticField, '/magnetometer_broadcaster/magnetic_field', received.append, 50)
        deadline = time.time() + 2.0
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.node.destroy_subscription(sub)

        self.assertGreater(
            len(received), 1,
            f'magnetic_field topic published only {len(received)} message(s) in 2 s'
            ' — expected continuous stream',
        )


    # ── Diagnostics ──────────────────────────────────────────────
    # TODO(rbscr)  diagnostics temporarily disabled

    #def test_diagnostics_topic_published(self):
    #    """Verify /diagnostics publishes at least one DiagnosticArray message."""
    #    from diagnostic_msgs.msg import DiagnosticArray
    #    msgs = _wait_for_topic(
    #        self.node, '/diagnostics', DiagnosticArray, timeout_sec=15.0)
    #    self.assertTrue(msgs, '/diagnostics not received within 15 s')

    #def test_diagnostics_status_level_valid(self):
    #    """Verify the BNO08X DiagnosticStatus level is a valid enum value."""
    #    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    #
    #    # Collect /diagnostics messages for up to 15 s; bno08x_diagnostics publishes
    #    # at 1 Hz and controller_manager also uses this topic, so we must search all
    #    # received messages rather than only the first one.
    #    received = []
    #    sub = self.node.create_subscription(
    #        DiagnosticArray, '/diagnostics', received.append, 10)
    #    deadline = time.time() + 15.0
    #    bno_status = None
    #    while time.time() < deadline:
    #        rclpy.spin_once(self.node, timeout_sec=0.1)
    #        for msg in received:
    #            for s in msg.status:
    #                if 'BNO08X' in s.name:
    #                    bno_status = s
    #                    break
    #            if bno_status:
    #                break
    #        if bno_status:
    #            break
    #    self.node.destroy_subscription(sub)

    #    self.assertIsNotNone(
    #        bno_status,
    #        'No BNO08X DiagnosticStatus found in /diagnostics within 15 s',
    #    )
    #    self.assertIn(
    #        bno_status.level,
    #        [DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR],
    #        f'Unexpected DiagnosticStatus level: {bno_status.level}',
    #    )
