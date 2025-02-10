import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import time

class LidarCartesianPlot(Node):

    def __init__(self):
        super().__init__('lidar_cartesian_plot')

        # Подписка на топик /lidar с типом LaserScan
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar',  # Имя топика LiDAR
            self.lidar_callback,
            10)

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

    def lidar_callback(self, msg):
        # Очистка предыдущих данных
        self.x_points.clear()
        self.y_points.clear()

        # Получение данных из LaserScan
        ranges = msg.ranges  # Массив расстояний
        angle_min = msg.angle_min  # Минимальный угол
        angle_increment = msg.angle_increment  # Шаг угла
        range_max = msg.range_max  # Максимальное расстояние
        print("MAX: ", range_max)

        # Генерация точек в декартовых координатах
        for i, distance in enumerate(ranges):
            # Проверка на корректность расстояния
            if not np.isinf(distance) and distance <= range_max:  # Игнорируем бесконечные и слишком большие значения
                angle = angle_min + i * angle_increment
                x = distance * np.cos(angle)  # Преобразование в декартовы координаты
                y = distance * np.sin(angle)
                print(distance, x, y)
                self.x_points.append(x)
                self.y_points.append(y)

        # Очистка графика и отрисовка новых данных
        self.ax.clear()
        self.ax.scatter(self.x_points, self.y_points, s=1, c='blue')  # Точки на 2D-графике
        self.ax.set_title("LiDAR Cartesian Plot (5x5 meters)")
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        self.ax.grid(True)
        self.ax.set_xlim(-3, 3)  # Ограничение по оси X
        self.ax.set_ylim(-3, 3)  # Ограничение по оси Y

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