import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.pub = self.create_publisher(String, 'key_cmd', 10)
        print("z=forward | s=backward | a=left | e=right | q=quit")

    def run(self):
        while rclpy.ok():
            try:
                key = input("> ")
            except UnicodeDecodeError:
                continue

            key = key.strip()
            if not key:
                continue

            key = key[0]

            if key == "q":
                break

            msg = String()
            msg.data = key
            self.pub.publish(msg)


def main(args=None):
    try:
        sys.stdin.reconfigure(encoding="utf-8", errors="ignore")
    except Exception:
        pass

    rclpy.init(args=args)
    node = KeyboardPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
