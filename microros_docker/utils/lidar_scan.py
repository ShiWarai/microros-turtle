import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import math
import tf2_ros
from rclpy.qos import QoSProfile, ReliabilityPolicy

class LidarCartesianPlot(Node):

    def __init__(self):
        super().__init__('lidar_cartesian_plot')

        # Set QoS profile
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            qos_profile)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.fig, self.ax = plt.subplots()
        self.ax.set_title("LiDAR Cartesian Plot (5x5 meters)")
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        self.ax.grid(True)
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)

        self.ax.axhline(0, color='black', linewidth=0.5)
        self.ax.axvline(0, color='black', linewidth=0.5)

        self.x_points = []
        self.y_points = []

    def quaternion_to_matrix(self, quaternion):
        """Преобразование кватерниона [x, y, z, w] в матрицу вращения 4x4"""
        x, y, z, w = quaternion
        
        # Вычисление элементов матрицы вращения
        xx = x * x
        xy = x * y
        xz = x * z
        xw = x * w
        yy = y * y
        yz = y * z
        yw = y * w
        zz = z * z
        zw = z * w

        # Создание матрицы 4x4
        matrix = np.array([
            [1 - 2*(yy + zz), 2*(xy - zw), 2*(xz + yw), 0],
            [2*(xy + zw), 1 - 2*(xx + zz), 2*(yz - xw), 0],
            [2*(xz - yw), 2*(yz + xw), 1 - 2*(xx + yy), 0],
            [0, 0, 0, 1]
        ])
        return matrix

    def lidar_callback(self, msg: LaserScan):
        self.x_points.clear()
        self.y_points.clear()

        ranges = msg.ranges
        intensity = msg.intensities
        angle_min = msg.angle_min
        angle_increment = 2 * math.pi / len(msg.ranges)
        range_max = msg.range_max
        range_min = msg.range_min

        try:
            trans = self.tf_buffer.lookup_transform('base_link', 'lidar', rclpy.time.Time())
            translation = [trans.transform.translation.x, 
                         trans.transform.translation.y, 
                         trans.transform.translation.z]
            rotation = [trans.transform.rotation.x,
                       trans.transform.rotation.y,
                       trans.transform.rotation.z,
                       trans.transform.rotation.w]
        except tf2_ros.LookupException:
            self.get_logger().info("Transform not found")
            return

        for i, distance in enumerate(ranges):
            if not np.isinf(distance) and distance <= range_max and distance >= range_min:
                angle = angle_min + i * angle_increment
                x = distance * np.cos(angle)
                y = distance * np.sin(angle)

                # Применение трансформации
                point = np.array([x, y, 0, 1])
                transform_matrix = self.quaternion_to_matrix(rotation)
                transform_matrix[0:3, 3] = translation  # Добавляем смещение
                transformed_point = np.dot(transform_matrix, point)
                
                self.x_points.append(transformed_point[0])
                self.y_points.append(transformed_point[1])

        self.ax.clear()
        self.ax.scatter(self.x_points, self.y_points, s=1, c='blue')
        self.ax.set_title("LiDAR Cartesian Plot")
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        self.ax.grid(True)
        self.ax.set_xlim(-range_max, range_max)
        self.ax.set_ylim(-range_max, range_max)

        self.ax.axhline(0, color='black', linewidth=0.5)
        self.ax.axvline(0, color='black', linewidth=0.5)

        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = LidarCartesianPlot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()