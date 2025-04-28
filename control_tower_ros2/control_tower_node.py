import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String, Bool
from geometry_msgs.msg import Point

from control_tower_ros2.double_ackermann import DoubleAckermannSteering as da
from control_tower_ros2.fixed_heading import FixedHeadingSteering as fh
from control_tower_ros2.rotate_in_place import RotateSteering as rip

MAX_VELOCITY = 2.0  # maximum velocity, in m/s
MAX_ACKERMANN_ANGLE = 45
# MIN_TURN_RADIUS = 3.0  # minimum ackermann turning radius, in m
MAX_WHEEL_ANGLE = 95  # maximum wheel angle, in degrees


class control_tower_node(Node):

    def __init__(self):
        super().__init__('control_tower_node')
        # Create a publisher for the Twist message on the 'cmd_vel' topic.
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        # Publisher for the switch states using an array of integers
        self.switch_publisher_ = self.create_publisher(
            Int32MultiArray, 'switch_states', 1)
        # Set up a timer to call update_callback periodically (e.g., every 0.1 seconds)
        self.timer = self.create_timer(0.01, self.update_callback)

        # Stick
        self.ry = 0  # Vroom vroom (CH3)
        self.rx = 0  # steering (CH1)
        self.ly = 0  # Throttle (CH2)
        self.lx = 0  # we dont know (CH4)

        # Switch
        self.sw_a = 0
        self.sw_b = 0  # broken
        self.sw_c = 0
        self.sw_d = 0

        # Create subscriptions
        self.sub_ch1 = self.create_subscription(
            Int32, 'ch1', self.callback_1, 1)
        self.sub_ch2 = self.create_subscription(
            Int32, 'ch2', self.callback_2, 1)
        self.sub_ch3 = self.create_subscription(
            Int32, 'ch3', self.callback_3, 1)
        self.sub_ch4 = self.create_subscription(
            Int32, 'ch4', self.callback_4, 1)

        self.sub_ch5 = self.create_subscription(
            Int32, 'ch5', self.callback_5, 1)
        self.sub_ch6 = self.create_subscription(
            Int32, 'ch6', self.callback_6, 1)
        self.sub_ch7 = self.create_subscription(
            Int32, 'ch7', self.callback_7, 1)
        self.sub_ch8 = self.create_subscription(
            Int32, 'ch8', self.callback_8, 1)

        self.frontleft_pub = self.create_publisher(
            Point, "frontleft/control", 10)
        self.frontright_pub = self.create_publisher(
            Point, "frontright/control", 10)
        self.backleft_pub = self.create_publisher(
            Point, "backleft/control", 10)
        self.backright_pub = self.create_publisher(
            Point, "backright/control", 10)

        self.direction_pub = self.create_publisher(Bool, "direction", 10)
        self.drive_mode_pub = self.create_publisher(String, "drive_mode", 10)
        self.control_state_pub = self.create_publisher(
            String, "control_state", 10)

    # Define separate callback functions for each channel

    def callback_1(self, msg): self.rx = msg.data
    def callback_2(self, msg): self.ly = msg.data
    def callback_3(self, msg): self.ry = msg.data
    def callback_4(self, msg): self.lx = msg.data

    def callback_5(self, msg): self.sw_a = msg.data
    def callback_6(self, msg): self.sw_d = msg.data
    def callback_7(self, msg): self.sw_c = msg.data
    def callback_8(self, msg): self.sw_b = msg.data

    def map_sw(self, value):
        if value == 1000:
            return 0
        elif value == 1500:
            return 1
        else:
            return 2

    def update_callback(self):

        direction = self.sw_a  # 1000: forward, 2000: reverse
        mode = self.sw_b  # 1000: teleop, 1500: off, 2000: auto
        drive = self.sw_c  # 1000: ackermann, 1500: fixed heading, 2000: rotate in place
        estop = self.sw_d  # 1000: on, 2000: off

        drive_mode_msg = String()
        control_state_msg = String()
        direction_msg = Bool()

        if estop == 1000:
            return  # do nothing

        if mode == 1000:  # teleop
            control_state_msg.data = "teleop"

            direction = 1 if direction == 2000 else 0
            if direction == 0:
                direction_msg.data = True
            else:
                direction_msg.data = False

            self.direction_pub.publish(direction_msg)

            rx_clamped = np.clip(self.rx, 1000, 2000)
            normalized_rx = (rx_clamped - 1500) / 500.0  # [-1, 1]
            ly_clamped = np.clip(self.ly, 1000, 2000)
            normalized_ly = (ly_clamped - 1500) / 500.0  # [-1, 1]

            if drive == 1000:
                # Double Ackermann
                velocity = normalized_ly * MAX_VELOCITY
                str_angle = normalized_rx * np.radians(MAX_ACKERMANN_ANGLE)
                if normalized_rx == 0:
                    turning_radius = float("inf")
                else:
                    turning_radius = abs(0.711 / np.tan(str_angle))
                    turning_radius *= (str_angle / abs(str_angle))
                drive_mode_msg.data = "ackermann"
                vehicle = da(velocity, turning_radius)
                self.publish_wheels(vehicle)

            elif drive == 1500:
                # Fixed Heading
                velocity = normalized_ly * MAX_VELOCITY
                angle = normalized_rx * MAX_WHEEL_ANGLE
                drive_mode_msg.data = "heading"
                vehicle = fh(velocity, angle)
                self.publish_wheels(vehicle)

            elif drive == 2000:
                # rotate in place
                drive_mode_msg.data = "rotate"
                rotate_velocity = normalized_rx * MAX_VELOCITY
                vehicle = rip(rotate_velocity)
                self.publish_wheels(vehicle)

            # Publish the Switch state
            sw_msg = Int32MultiArray()
            sw_msg.data = [
                self.map_sw(self.sw_a),
                self.map_sw(self.sw_b),
                self.map_sw(self.sw_c),
                self.map_sw(self.sw_d)
            ]
            self.switch_publisher_.publish(sw_msg)

        elif mode == 1500:
            control_state_msg.data = "off"

        elif mode == 2000:
            control_state_msg.data = "auto"

        self.control_state_pub.publish(control_state_msg)
        self.drive_mode_pub.publish(drive_mode_msg)

    def publish_wheels(self, vehicle: da):
        frontleft_ctrl = Point()
        frontright_ctrl = Point()
        backleft_ctrl = Point()
        backright_ctrl = Point()

        frontleft_ctrl.x = vehicle.v_f_left * 1.0
        frontleft_ctrl.z = vehicle.theta_f_left * 1.0
        frontright_ctrl.x = vehicle.v_f_right * 1.0
        frontright_ctrl.z = vehicle.theta_f_right * 1.0
        backleft_ctrl.x = vehicle.v_r_left * 1.0
        backleft_ctrl.z = vehicle.theta_r_left * 1.0
        backright_ctrl.x = vehicle.v_r_right * 1.0
        backright_ctrl.z = vehicle.theta_r_right * 1.0

        self.frontleft_pub.publish(frontleft_ctrl)
        self.frontright_pub.publish(frontright_ctrl)
        self.backleft_pub.publish(backleft_ctrl)
        self.backright_pub.publish(backright_ctrl)


def main(args=None):
    rclpy.init(args=args)
    node = control_tower_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
