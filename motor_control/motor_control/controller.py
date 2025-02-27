import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

# Controller Node
class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        
        # Declare parameters for control gains
        self.declare_parameter('kp', 0.003)
        self.declare_parameter('ki', 2.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('sample_time', 0.01)
        
        # Control variables
        self.error_prev = 0.0
        self.integral = 0.0
        
        # Create publishers and subscribers
        self.motor_input_pub = self.create_publisher(Float32, 'motor_input_u', 10)
        self.set_point_sub = self.create_subscription(Float32, 'set_point', self.set_point_callback, 10)
        self.motor_speed_sub = self.create_subscription(Float32, 'motor_speed_y', self.motor_speed_callback, 10)
        
        # Timer
        self.timer = self.create_timer(self.get_parameter('sample_time').value, self.control_loop)
        
        # Messages
        self.set_point = 0.0
        self.motor_speed = 0.0
        
        self.get_logger().info("Controller Node Started \U0001F680")
        
        # Parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)
    
    def set_point_callback(self, msg):
        self.set_point = msg.data
    
    def motor_speed_callback(self, msg):
        self.motor_speed = msg.data
    
    def control_loop(self):
        # Read parameters
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        sample_time = self.get_parameter('sample_time').value
        
        # Compute error
        error = self.set_point - self.motor_speed
        
        # PID calculations
        self.integral += error * sample_time
        derivative = (error - self.error_prev) / sample_time
        
        control_signal = kp * error + ki * self.integral + kd * derivative
        
        # Publish control signal
        msg = Float32()
        msg.data = control_signal
        self.motor_input_pub.publish(msg)
        
        # Update previous error
        self.error_prev = error
    
    def parameters_callback(self, params):
        return SetParametersResult(successful=True)

# Main
def main(args=None):
    rclpy.init(args=args)

    node = Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

#Execute Node
if __name__ == '__main__':
    main()
