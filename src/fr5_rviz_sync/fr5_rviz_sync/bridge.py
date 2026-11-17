"""One-way state bridge. Never imports the vendor SDK or a command service."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from fairino_msgs.msg import RobotNonrtState
from .conversion import JOINT_NAMES, degrees_to_joint_positions


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge', namespace='fr5_live')
        self.declare_parameter('input_topic', '/nonrt_state_data')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('stale_timeout_sec', 2.0)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.stale_timeout = float(self.get_parameter('stale_timeout_sec').value)
        if (not math.isfinite(rate) or not 1 <= rate <= 500 or
                not math.isfinite(self.stale_timeout) or self.stale_timeout <= 0):
            raise ValueError('Invalid feedback rate or stale timeout')
        self.interval = 1.0 / rate
        self.last_publish = -float('inf')
        self.last_receive = None
        self.started = time.monotonic()
        self.last_invalid_warning = -float('inf')
        self.last_health_log = -float('inf')
        self.stale = False
        self.received_count = 0
        self.published_count = 0
        self.latest_joints = None
        self.latest_error = None
        self.publisher = self.create_publisher(JointState, '/fr5_live/joint_states', 10)
        self.subscription = self.create_subscription(
            RobotNonrtState, self.get_parameter('input_topic').value,
            self.on_state, qos_profile_sensor_data)
        self.watchdog = self.create_timer(1.0, self.check_state)
        self.get_logger().info(
            'READ ONLY: /nonrt_state_data (degrees) -> /fr5_live/joint_states (radians). '
            'No enable, motion or IO commands; stamps are local receive time.')

    def on_state(self, state):
        now = time.monotonic()
        try:
            degrees = [getattr(state, f'j{i}_cur_pos') for i in range(1, 7)]
            positions = degrees_to_joint_positions(degrees)
        except (ValueError, TypeError, AttributeError, OverflowError) as error:
            if now - self.last_invalid_warning > 2.0:
                self.get_logger().warning(f'Dropping invalid joint feedback: {error}')
                self.last_invalid_warning = now
            return
        first = self.last_receive is None
        self.last_receive = now
        self.latest_joints = degrees
        self.latest_error = (state.rbt_main_code, state.rbt_sub_code)
        self.received_count += 1
        if first or self.stale:
            self.get_logger().info('Live feedback received; RViz can follow the physical robot.')
            self.stale = False
        if now - self.last_publish < self.interval:
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = list(positions)
        # Velocity and effort remain empty: no invented measurements.
        self.publisher.publish(message)
        self.last_publish = now
        self.published_count += 1

    def check_state(self):
        now = time.monotonic()
        reference = self.last_receive if self.last_receive is not None else self.started
        age = now - reference
        if age > self.stale_timeout:
            if not self.stale or now - self.last_health_log >= 5.0:
                self.get_logger().warning(
                    f'NO LIVE FEEDBACK for {age:.1f}s. RViz may show an old pose; '
                    'check the driver, robot network and ROS domain. No synthetic states are published.')
                self.last_health_log = now
            self.stale = True
        elif self.last_receive is not None and now - self.last_health_log >= 5.0:
            self.get_logger().info(
                f'LIVE rx={self.received_count} published={self.published_count} '
                f'joints_deg={[round(x, 3) for x in self.latest_joints]} '
                f'robot_error={self.latest_error}')
            self.last_health_log = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = JointStateBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
