import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class CmdSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_subscriber')
        self.ser = serial.Serial('/dev/ttyACM0', 57600, timeout=0.1)
        self.create_subscription(String, 'key_cmd', self.callback, 10)

    def callback(self, msg):
        key = msg.data

        if key == 'z':        # forward
            cmd = "V -50 -50\n"
        elif key == 's':      # backward
            cmd = "V 50 50\n"
        elif key == 'a':      # left
            cmd = "V 50 -50\n"
        elif key == 'e':      # right
            cmd = "V -50 50\n"
        elif key == 'x':      # stop
            cmd = "V 0 0\n"
        else:
            return

        self.ser.write(cmd.encode())

def main():
    rclpy.init()
    node = CmdSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

