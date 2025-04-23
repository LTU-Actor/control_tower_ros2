import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from geometry_msgs.msg import Point

from control_tower_ros2.double_ackermann import DoubleAckermannSteering as da
from control_tower_ros2.fixed_heading import FixedHeadingSteering as fh
from control_tower_ros2.rotate_in_place import RotateSteering as rip


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
        
        self.frontleft_pub =  self.create_publisher(Point, "frontleft/control", 10)
        self.frontright_pub = self.create_publisher(Point, "frontright/control", 10)
        self.backleft_pub =   self.create_publisher(Point, "backleft/control", 10)
        self.backright_pub =  self.create_publisher(Point, "backright/control", 10)
        

    # Define separate callback functions for each channel
    def callback_1(self, msg): self.rx = msg.data
    def callback_2(self, msg): self.ly = msg.data
    def callback_3(self, msg): self.ry = msg.data
    def callback_4(self, msg): self.lx = msg.data

    def callback_5(self, msg): self.sw_a = msg.data
    def callback_6(self, msg): self.sw_b = msg.data
    def callback_7(self, msg): self.sw_c = msg.data
    def callback_8(self, msg): self.sw_d = msg.data

    def map_sw(self, value):
        if value == 1000:
            return 0
        elif value == 1500:
            return 1
        else:
            return 2

    def update_callback(self):

        # 0: double Ackermann, 1: Fixed Heading, 2: edu-bot test mode
        self.drive_mode = self.sw_c

        if self.drive_mode == 2000:
            # Double Ackermann
            # L: Length (m), W: Width (m), max_speed: max speed (max speed is not used in the current implementation)
            vehicle = da(self.lx, self.ly)
            self.publish_wheels(vehicle)
            # TODO: Publish wheel angles/velocities from vehicle object
            # vehicle.theta_f_left
            # vehicle.theta_f_right
            # vehicle.theta_r_left
            # vehicle.theta_r_right
            # vehicle.v_f_left
            # vehicle.v_f_right
            # vehicle.v_r_left
            # vehicle.v_r_right

            # Debugging
            # self.get_logger().info(f"Published Wheel Angles: {vehicle.theta_f_left}, {vehicle.theta_f_right}, {vehicle.theta_r_left}, {vehicle.theta_r_right}")
            # self.get_logger().info(f"Published Wheel Velocities: {vehicle.v_f_left}, {vehicle.v_f_right}, {vehicle.v_r_left}, {vehicle.v_r_right}")

        elif self.drive_mode == 1500:
            # Fixed Heading
            vehicle = fh(self.lx, self.ly)
            self.publish_wheels(vehicle)
            
        elif self.drive_mode == 1000:
            # rotate in place
            vehicle = rip(self.lx, self.ly)
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
        # Debugging
        # self.get_logger().info(f"Published Switch States: {sw_msg.data}")

    def publish_wheels(self, vehicle : da):
        frontleft_ctrl =  Point()
        frontright_ctrl = Point()
        backleft_ctrl =   Point()
        backright_ctrl =  Point()
        
        frontleft_ctrl.x =  vehicle.v_f_left      * 1.0
        frontleft_ctrl.z = vehicle.theta_f_left * 1.0
        frontright_ctrl.x =  vehicle.v_f_right * 1.0
        frontright_ctrl.z = vehicle.theta_f_right * 1.0
        backleft_ctrl.x =  vehicle.v_r_left * 1.0
        backleft_ctrl.z = vehicle.theta_r_left * 1.0
        backright_ctrl.x =  vehicle.v_r_right * 1.0
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
