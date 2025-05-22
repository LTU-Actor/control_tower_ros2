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
        # Set up a timer to call update_callback periodically (e.g., every 0.1 seconds)
        self.timer = self.create_timer(0.01, self.update_callback)

        # Stick
        self.ry = 0  # Vroom vroom (CH3)
        self.rx = 0  # steering (CH1)
        self.ly = 0  # Throttle (CH2)
        self.lx = 0  # we dont know (CH4)

        # Switch
        self.sw_a = 0
        self.sw_b = 0
        self.sw_c = 0
        self.sw_d = 0

        self.direction = False # False = forward, True = backward
        self.control_state = "off"
        self.drive_mode = "ackermann"
        self.last_cmd_vel : Twist = None

        # Create subscriptions
        self.create_subscription(Bool, "set_direction", self.direction_cb, 1)
        self.create_subscription(String, "set_drive_mode", self.drive_mode_cb, 1)
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_cb, 1)
        
        self.control_state_pub = self.create_publisher(
            String, "control_state", 10)
        self.drive_mode_pub = self.create_publisher(String, "drive_mode", 10)
        self.direction_pub = self.create_publisher(Bool, "direction", 10)



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
        

    # Define separate callback functions for each channel

    def callback_1(self, msg): self.rx = msg.data
    def callback_2(self, msg): self.ly = msg.data
    def callback_3(self, msg): self.ry = msg.data
    def callback_4(self, msg): self.lx = msg.data

    def callback_5(self, msg): self.sw_a = msg.data
    def callback_6(self, msg): self.sw_d = msg.data
    def callback_7(self, msg): self.sw_c = msg.data
    def callback_8(self, msg): self.sw_b = msg.data

    def direction_cb(self, msg):
        if self.control_state == "auto":
            self.direction = msg.data

    def drive_mode_cb(self, msg):
        if self.control_state == "auto":
            mode = msg.data
            if mode in ["ackermann", "heading", "rotate"]:
                self.drive_mode = mode
            else:
                self.get_logger().warn(f"Got invalid driving mode: {mode}")

    def cmd_vel_cb(self, msg):
        if self.control_state == "auto":
            self.last_cmd_vel = msg
        else:
            self.last_cmd_vel = None


    def update_callback(self):

        mode = self.sw_b  # 1000: teleop, 1500: off, 2000: auto
        estop = self.sw_d  # 1000: on, 2000: off
        vehicle = None

        if estop == 1000:
            return  # do nothing

        if mode == 1000:  # teleop
            self.control_state = "teleop"

            self.direction = self.sw_a == 2000

            if self.sw_c == 1000:
                self.drive_mode = "ackermann"
            elif self.sw_c == 1500:
                self.drive_mode = "heading"
            elif self.sw_c == 2000:
                self.drive_mode = "rotate"

            rx_clamped = np.clip(self.rx, 1000, 2000)
            normalized_rx = (rx_clamped - 1500) / 500.0  # [-1, 1]
            ly_clamped = np.clip(self.ly, 1000, 2000)
            normalized_ly = (ly_clamped - 1500) / 500.0  # [-1, 1]

            if self.drive_mode == "ackermann":
                # Double Ackermann
                velocity = normalized_ly * MAX_VELOCITY
                if (not self.direction and velocity < 0) or self.direction and velocity > 0:
                    velocity = 0
                str_angle = normalized_rx * np.radians(MAX_ACKERMANN_ANGLE)
                if normalized_rx == 0:
                    turning_radius = float("inf")
                else:
                    turning_radius = abs(0.711 / np.tan(str_angle))
                    turning_radius *= (str_angle / abs(str_angle))
                vehicle = da(velocity, turning_radius)
                

            elif self.drive_mode == "heading":
                # Fixed Heading
                velocity = normalized_ly * MAX_VELOCITY
                if (not self.direction and velocity < 0) or self.direction and velocity > 0:
                    velocity = 0
                angle = normalized_rx * np.radians(MAX_WHEEL_ANGLE)
                vehicle = fh(velocity, angle)
                

            elif self.drive_mode == "rotate":
                # rotate in place
                rotate_velocity = normalized_rx * MAX_VELOCITY
                if (not self.direction and rotate_velocity < 0) or self.direction and rotate_velocity > 0:
                    rotate_velocity = 0
                vehicle = rip(rotate_velocity)
                

        elif mode == 1500:
            self.control_state = "off"

        elif mode == 2000:
            self.control_state = "auto"
            if self.last_cmd_vel:
                if self.drive_mode == "ackermann":
                    velocity = max(min(self.last_cmd_vel.linear.x, MAX_VELOCITY), -1 * MAX_VELOCITY)
                    if (not self.direction and velocity < 0) or self.direction and velocity > 0:
                        velocity = 0
                    str_angle = max(min(self.last_cmd_vel.angular.z, np.radians(MAX_ACKERMANN_ANGLE)), -1 * np.radians(MAX_ACKERMANN_ANGLE))
                    if str_angle == 0:
                        turning_radius = float("inf")
                    else:
                        turning_radius = abs(0.711 / np.tan(str_angle))
                        turning_radius *= (str_angle / abs(str_angle))
                    vehicle = da(velocity, turning_radius)
                    

                elif self.drive_mode == "heading":
                    velocity = max(min(self.last_cmd_vel.linear.x, MAX_VELOCITY), -1 * MAX_VELOCITY)
                    if (not self.direction and velocity < 0) or self.direction and velocity > 0:
                        velocity = 0
                    angle = max(min(self.last_cmd_vel.angular.z, np.radians(MAX_WHEEL_ANGLE)), -1 * np.radians(MAX_WHEEL_ANGLE))
                    vehicle = fh(velocity, angle)
                    

                elif self.drive_mode == "rotate":
                    rotate_velocity = max(min(self.last_cmd_vel.angular.z, MAX_VELOCITY), -1 * MAX_VELOCITY)
                    if (not self.direction and rotate_velocity < 0) or self.direction and rotate_velocity > 0:
                        rotate_velocity = 0
                    vehicle = rip(rotate_velocity)
                self.last_cmd_vel = None
                    

        control_state_msg = String()
        control_state_msg.data = self.control_state
        self.control_state_pub.publish(control_state_msg)
        
        drive_mode_msg = String()
        drive_mode_msg.data = self.drive_mode
        self.drive_mode_pub.publish(drive_mode_msg)
        
        direction_msg = Bool()
        direction_msg.data = not self.direction
        self.direction_pub.publish(direction_msg)
        
        if(vehicle):
            self.publish_wheels(vehicle)

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
