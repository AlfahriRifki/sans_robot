import rclpy
from geometry_msgs.msg import Twist

def main():
    rclpy.init()

    node = rclpy.create_node('velocity_publisher')

    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    while rclpy.ok():
        try:
            linear_vel = float(input("Enter linear velocity (m/s): "))
        except ValueError:
            print("Invalid input. Please enter a numeric value.")
            continue

        msg = Twist()
        msg.linear.x = linear_vel

        publisher.publish(msg)
        node.get_logger().info('Publishing linear velocity command: %s m/s' % msg.linear.x)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
