import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


class CANBusSimulator(Node):
    def __init__(self):
        super().__init__('can_bus_simulator')

        # 发布车速 → FSD controller_node  (#4 /localization/velocity)
        self.twist_pub = self.create_publisher(
            TwistStamped, '/localization/velocity', 50)

        # 订阅 ground truth → 提取车速  (#1 /sim/ground_truth)
        self.gt_sub = self.create_subscription(
            Odometry, '/sim/ground_truth',
            self.ground_truth_callback, 50)

        self.get_logger().info(
            'CANBusSimulator started: /sim/ground_truth -> /localization/velocity')

    def ground_truth_callback(self, msg: Odometry):
        twist = TwistStamped()
        twist.header.stamp = msg.header.stamp
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x = msg.twist.twist.linear.x
        twist.twist.linear.y = msg.twist.twist.linear.y
        twist.twist.linear.z = msg.twist.twist.linear.z
        twist.twist.angular.x = msg.twist.twist.angular.x
        twist.twist.angular.y = msg.twist.twist.angular.y
        twist.twist.angular.z = msg.twist.twist.angular.z
        self.twist_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = CANBusSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down CANBusSimulator...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
