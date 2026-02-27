# This code enables teleop_twist_keyboard to raise an lower the forklift fork.
# 
# The fork is controlled by the forward_command_controller/ForwardCommandController
# and listens on /velocity_control/commands for std_msgs/msg/Float64MultiArray for
# a 1 element array with a number.  A positive number is the speed up, negative number
# is the speed down, and 0 means stop.
#
# teleop_twist_keyboard uses the t and b keys for up/down on the z-axis.  The value
# is published as a twist message and is incompatible with velocity_control.
#
# This code intercepts twist messages, reads the twist z-axis velocity command,
# repacks the value as Float64MultiArray, and publishes it to /velocity_control/commands. 
#
#
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class TwistToMultiArray(Node):
    def __init__(self):
        super().__init__('twist_to_multiarray')
        # Subscribe to the raw teleop output
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.listener_callback, 10)
        # Publish the Float64MultiArray
        self.publisher = self.create_publisher(
            Float64MultiArray, '/velocity_control/commands', 10)

    def listener_callback(self, msg):
        # Extract linear z-axis from Twist
        z_value = msg.linear.z
        
        # Create MultiArray message
        multiarray_msg = Float64MultiArray()
        # Example: [z_value] or populate with more data
        multiarray_msg.data = [z_value] 
        
        self.publisher.publish(multiarray_msg)
        self.get_logger().info(f'Publishing: {multiarray_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = TwistToMultiArray()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

