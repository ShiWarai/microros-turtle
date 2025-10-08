#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomToTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')

        # Создаём broadcaster для публикации трансформаций
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Подписываемся на топик /odom
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.get_logger().info("Odom to TF broadcaster started")

    def odom_callback(self, msg):
        # Создаём сообщение TransformStamped
        t = TransformStamped()

        # Устанавливаем заголовок (время и frame_id)
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        # Позиция из сообщения одометрии
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Ориентация из сообщения одометрии
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        # Публикуем трансформацию
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug(f"Published transform: x={t.transform.translation.x}, y={t.transform.translation.y}, theta={t.transform.rotation.z}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()