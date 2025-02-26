import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

# Class Definition
class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node')

        # Declare parameters for signal type and frequency
        self.declare_parameter('signal_type', 'sine')  # sine, square
        self.declare_parameter('amplitude', 2.0)
        self.declare_parameter('frequency', 1.0)
        self.timer_period = 0.1  # seconds

        # Create a publisher and timer for the signal
        self.signal_publisher = self.create_publisher(Float32, 'set_point', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_cb)
        
        # Messages and variables
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()
        self.get_logger().info("SetPoint Node Started \U0001F680")

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)

    # Timer Callback: Generate and Publish Signal
    def timer_cb(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        amplitude = self.get_parameter('amplitude').value
        frequency = self.get_parameter('frequency').value
        signal_type = self.get_parameter('signal_type').value
        
        if signal_type == 'sine':
            self.signal_msg.data = amplitude * np.sin(2 * np.pi * frequency * elapsed_time)
        elif signal_type == 'square':
            self.signal_msg.data = amplitude * np.sign(np.sin(2 * np.pi * frequency * elapsed_time))
        else:
            self.get_logger().warn(f"Unknown signal type: {signal_type}, defaulting to sine.")
            self.signal_msg.data = amplitude * np.sin(2 * np.pi * frequency * elapsed_time)

        self.signal_publisher.publish(self.signal_msg)
    
    def parameters_callback(self, params):
        return SetParametersResult(successful=True)

# Main
def main(args=None):
    rclpy.init(args=args)
    node = SetPointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
