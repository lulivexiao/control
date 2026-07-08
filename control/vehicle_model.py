import math

import rclpy
from rclpy.node import Node
from autoware_msgs.msg import Command
from nav_msgs.msg import Odometry


class VehicleModel(Node):
    def __init__(self):
        super().__init__('vehicle_model')

        # 参数
        self.declare_parameter('wheel_base', 1.53)       # m
        self.declare_parameter('max_steer_angle', 25.0)   # 度
        self.declare_parameter('dt', 0.02)                # 50Hz
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_steer = math.radians(
            self.get_parameter('max_steer_angle').value)
        self.dt = self.get_parameter('dt').value

        # 状态
        self.x = self.get_parameter('start_x').value
        self.y = self.get_parameter('start_y').value
        self.yaw = self.get_parameter('start_yaw').value
        self.speed = 0.0
        self.angle = 0.0       # 度

        # 订阅控制指令 (#6 /control/command)
        self.sub = self.create_subscription(
            Command, '/control/command', self.on_command, 50)

        # 发布 ground truth (#1 /sim/ground_truth)
        self.pub = self.create_publisher(
            Odometry, '/sim/ground_truth', 50)

        # 定时更新 50Hz
        self.timer = self.create_wall_timer(self.dt, self.step)

    def on_command(self, msg: Command):
        self.speed = msg.speed          # m/s
        self.angle = msg.angle          # 度

    def step(self):
        # 限幅转向角
        steer = max(-self.max_steer,
                    min(math.radians(self.angle), self.max_steer))

        # 自行车运动学更新
        self.x += self.speed * math.cos(self.yaw) * self.dt
        self.y += self.speed * math.sin(self.yaw) * self.dt
        self.yaw += self.speed * math.tan(steer) / self.wheel_base * self.dt

        now = self.get_clock().now().to_msg()

        # 发布 Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = self.speed
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.speed * math.tan(steer) / self.wheel_base
        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down VehicleModel...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()