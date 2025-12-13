import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class CmdSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_subscriber')
        self.ser = serial.Serial('/dev/ttyACM0', 57600, timeout=0.1)
        self.create_subscription(String, 'key_cmd', self.callback, 10)
        self.get_logger().info("Ready to receive commands.")

    def callback(self, msg):
        key = msg.data

        if key == 'z':
            cmd = "D 50 50 1\n"
        elif key == 's':
            cmd = "D -50 -50 1\n"
        elif key == 'a':
            cmd = "D 50 -50 1\n"
        elif key == 'e':
            cmd = "D -50 50 1\n"
        else:
            return
        
        self.ser.write(cmd.encode())
        self.get_logger().info(f"Sent: {cmd.strip()}")

def main():
    rclpy.init()
    node = CmdSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
