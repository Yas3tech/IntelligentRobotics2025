import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class CmdSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_subscriber')
        # Port série de l'OpenCR
        self.ser = serial.Serial('/dev/ttyACM0', 57600, timeout=0.1)
        # Abonnement au topic venant du PC
        self.create_subscription(String, 'key_cmd', self.callback, 10)

    def callback(self, msg):
        key = msg.data

        if key == 'z':
            cmd = "D 50 50 1\n"       # avant
        elif key == 's':
            cmd = "D -50 -50 1\n"     # arrière
        elif key == 'a':
            cmd = "D 50 -50 1\n"      # gauche
        elif key == 'e':
            cmd = "D -50 50 1\n"      # droite
        else:
            return  # touche inconnue → on ne fait rien

        self.ser.write(cmd.encode())

def main():
    rclpy.init()
    node = CmdSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
