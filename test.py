import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopTwistKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.key_bindings = {
            'i': (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            'k': (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            'j': (0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            'l': (0.0, 0.0, 0.0, 0.0, 0.0, -1.0),
            'u': (1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            'o': (-1.0, 0.0, 0.0, 0.0, 0.0, -1.0),
            ',': (0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            '.': (0.0, 0.0, 0.0, 0.0, -1.0, 0.0),
        }

    def run(self):
        while True:
            key = input('Enter a command (i, k, j, l, u, o, ,, .): ')
            if key in self.key_bindings:
                twist = Twist()
                twist.linear.x = self.key_bindings[key][0]
                twist.linear.y = self.key_bindings[key][1]
                twist.linear.z = self.key_bindings[key][2]
                twist.angular.x = self.key_bindings[key][3]
                twist.angular.y = self.key_bindings[key][4]
                twist.angular.z = self.key_bindings[key][5]
                self.publisher_.publish(twist)
            else:
                print('Invalid key command!')

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopTwistKeyboard()
    teleop_node.run()
    rclpy.spin(teleop_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
