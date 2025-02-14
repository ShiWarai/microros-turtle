import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import math
import tf2_ros
import tf_transformations

class LidarCartesianPlot(Node):

    def __init__(self):
        super().__init__('lidar_cartesian_plot')

        # Подписка на топик /lidar с типом LaserScan
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar',  # Имя топика LiDAR
            self.lidar_callback,
            10)

        # Инициализация трансформаций
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Инициализация графика
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("LiDAR Cartesian Plot (5x5 meters)")
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        self.ax.grid(True)
        self.ax.set_xlim(-5, 5)  # Ограничение по оси X
        self.ax.set_ylim(-5, 5)  # Ограничение по оси Y

        # Добавление линий для квадрантов
        self.ax.axhline(0, color='black', linewidth=0.5)  # Горизонтальная линия (ось X)
        self.ax.axvline(0, color='black', linewidth=0.5)  # Вертикальная линия (ось Y)

        # Для хранения данных
        self.x_points = []
        self.y_points = []

    def lidar_callback(self, msg: LaserScan):
        # Очистка предыдущих данных
        self.x_points.clear()
        self.y_points.clear()

        # Получение данных из LaserScan
        ranges = msg.ranges  # Массив расстояний
        intensity = msg.intensities
        angle_min = msg.angle_min  # Минимальный угол
        angle_increment = 2 * math.pi / len(msg.ranges) # Шаг угла
        range_max = msg.range_max  # Максимальное расстояние
        range_min = msg.range_min

        # Получение трансформации между base и lidar_frame
        try:
            trans = self.tf_buffer.lookup_transform('base', 'lidar_frame', rclpy.time.Time())
            translation = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            rotation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
        except tf2_ros.LookupException:
            self.get_logger().info("Transform not found")
            return

        # Генерация точек в декартовых координатах
        for i, distance in enumerate(ranges):
            # Проверка на корректность расстояния
            if msg.intensities[i] >= 5 and not np.isinf(distance) and distance <= range_max and distance >= range_min:
                angle = angle_min + i * angle_increment
                x = distance * np.cos(angle)  # Преобразование в декартовы координаты
                y = distance * np.sin(angle)

                # Применение трансформации
                point = np.array([x, y, 0, 1])
                transform_matrix = tf_transformations.quaternion_matrix(rotation)
                transform_matrix[:3, 3] = translation
                transformed_point = np.dot(transform_matrix, point)
                
                self.x_points.append(transformed_point[0])
                self.y_points.append(transformed_point[1])

        # Очистка графика и отрисовка новых данных
        self.ax.clear()
        self.ax.scatter(self.x_points, self.y_points, s=1, c='blue')  # Точки на 2D-графике
        self.ax.set_title("LiDAR Cartesian Plot")
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        self.ax.grid(True)
        self.ax.set_xlim(-range_max, range_max)  # Ограничение по оси X
        self.ax.set_ylim(-range_max, range_max)  # Ограничение по оси Y

        # Добавление линий для квадрантов
        self.ax.axhline(0, color='black', linewidth=0.5)  # Горизонтальная линия (ось X)
        self.ax.axvline(0, color='black', linewidth=0.5)  # Вертикальная линия (ось Y)

        # Обновление графика
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = LidarCartesianPlot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
