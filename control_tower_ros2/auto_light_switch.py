import gpiod
from gpiod.line import Direction, Value

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

AUTO_GPIO_PIN = 20
OTHER_GPIO_PIN = 21


class LightSwitch(Node):
    
    control_state = ""
    
    def __init__(self):
        super().__init__("auto_light_switch")
        self.create_subscription(String, "control_state", self.control_state_cb, 1)
        
        self.gpio_request = gpiod.request_lines("/dev/gpiochip0", 
                                                consumer="auto_light_switch", 
                                                config={
                                                    AUTO_GPIO_PIN: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE),
                                                    OTHER_GPIO_PIN: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE),
                                                })
        
        self.do_lights()
        
        
    def control_state_cb(self, msg : String):
        self.control_state = msg.data
        
    def do_lights(self):
        rate = self.create_rate(10)
        while rclpy.ok():
            if self.control_state == "auto":
                self.gpio_request.set_value(AUTO_GPIO_PIN, Value.ACTIVE)
                self.gpio_request.set_value(OTHER_GPIO_PIN, Value.INACTIVE)
            else:
                self.gpio_request.set_value(AUTO_GPIO_PIN, Value.INACTIVE)
                self.gpio_request.set_value(OTHER_GPIO_PIN, Value.ACTIVE)
            rate.sleep()
            
            
def main(args=None):
    rclpy.init(args=args)
    node = LightSwitch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
        
    