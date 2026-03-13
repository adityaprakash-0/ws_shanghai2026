import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from studica_control.srv import SetData
import sys, select, termios, tty

# Save terminal settings globally so get_key can use them
settings = termios.tcgetattr(sys.stdin)

class TeleopNode(Node):
    def __init__(self):
        super().__init__('custom_teleop')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.imu_client = self.create_client(SetData, '/imu/get_imu_data')
        self.titan_client = self.create_client(SetData, '/titan0/titan_cmd')
        
        # --- Hardcoded Speeds ---
        self.linear_speed = 0.3
        self.angular_speed = 0.25

        self.get_logger().info("Teleop Ready. Keys: W (fwd), S (rev), A (left), D (right), SPACE (stop) | R (reset encoder) | G (reset imu). CTRL+C to quit.")

    def publish_twist(self, lin, ang):
        twist = Twist()
        twist.linear.x = float(lin)
        twist.angular.z = float(ang)
        self.cmd_pub.publish(twist)

    def reset_imu(self):
        self.get_logger().info("Sending Reset Command to IMU....")
        # Reset IMU
        if self.imu_client.service_is_ready():
            req_imu = SetData.Request()
            req_imu.params = "reset"
            self.imu_client.call_async(req_imu)
        else:
            self.get_logger().warn("IMU service is not ready. Skipping IMU reset.")

    def reset_encoders(self):
        self.get_logger().info("Sending Reset Command to Encoders....")
        if self.titan_client.service_is_ready():
            for i in range(4):
                req_enc = SetData.Request()
                req_enc.params = "reset_encoder"
                req_enc.initparams.n_encoder = i
                self.titan_client.call_async(req_enc)
        else:
            self.get_logger().warn("Titan service is not ready. Skipping Encoder reset.")

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    try:
        while rclpy.ok():
            key = get_key()
            if key == 'w':
                node.publish_twist(node.linear_speed, 0.0)
            elif key == 's':
                node.publish_twist(-node.linear_speed, 0.0)
            elif key == 'a':
                node.publish_twist(0.0, node.angular_speed)
            elif key == 'd':
                node.publish_twist(0.0, -node.angular_speed)
            elif key == ' ' or key == 'x':
                node.publish_twist(0.0, 0.0)
            elif key == 'r':
                node.reset_encoders()
            elif key == 'g':
                node.reset_imu()
            elif key == '\x03': # CTRL+C
                break
    except Exception as e:
        print(e)
    finally:
        node.publish_twist(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()