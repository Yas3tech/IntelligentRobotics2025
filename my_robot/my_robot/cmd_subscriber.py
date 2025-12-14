import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

KEYMAP = {
    'z': "D 50 50 1",     # forward
    'a': "D 50 -50 1",    # left
    'e': "D -50 50 1",    # right
    's': "D -50 -50 1",   # backward
}

class CmdSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_subscriber')
        self.ser = serial.Serial('/dev/ttyACM0', 57600, timeout=0.1)
        self.create_subscription(String, 'key_cmd', self.cb, 10)
        self.get_logger().info("cmd_subscriber ready (D commands)")

    def cb(self, msg):
        key = msg.data
        if key in KEYMAP:
            cmd = KEYMAP[key]
            self.ser.write((cmd + "\n").encode())
            self.get_logger().info(f"> {cmd}")


def main():
    rclpy.init()
    node = CmdSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
