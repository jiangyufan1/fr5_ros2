"""Read-only subscriber; does not start a driver or send robot commands."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from fairino_msgs.msg import RobotNonrtState

class StateEcho(Node):
    def __init__(self):
        super().__init__('fr5_state_echo')
        self.declare_parameter('state_topic', '/nonrt_state_data')
        self.declare_parameter('print_period_sec', 1.0)
        topic = self.get_parameter('state_topic').value
        self.period = max(float(self.get_parameter('print_period_sec').value), 0.1)
        self.last_print = -float('inf')
        self.last_receive = None
        self.started = time.monotonic()
        self.subscription = self.create_subscription(
            RobotNonrtState, topic, self.on_state, qos_profile_sensor_data)
        self.timer = self.create_timer(5.0, self.check_connection)
        self.get_logger().info(f'Listening on {topic}; no robot commands are sent.')

    def on_state(self, msg):
        now = time.monotonic()
        self.last_receive = now
        if now - self.last_print < self.period:
            return
        self.last_print = now
        joints = [getattr(msg, f'j{i}_cur_pos') for i in range(1, 7)]
        self.get_logger().info(
            f'joints_deg={[round(v, 3) for v in joints]} '
            f'DI_low={msg.dgt_input_l:#04x} DI_high={msg.dgt_input_h:#04x} '
            f'main_error={msg.rbt_main_code} sub_error={msg.rbt_sub_code}')

    def check_connection(self):
        reference = self.last_receive if self.last_receive is not None else self.started
        if time.monotonic() - reference > 5:
            self.get_logger().warning('No recent state messages. Check driver, robot network and ROS_DOMAIN_ID.')

def main(args=None):
    rclpy.init(args=args)
    node = StateEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
