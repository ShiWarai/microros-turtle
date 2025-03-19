import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np

class OdomVisualizer(Node):
    def __init__(self):
        super().__init__('odom_visualizer')
        # Подписываемся на тему /odom
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry',
            self.odom_callback,
            10)
        self.subscription  # Предотвращаем предупреждение о неиспользуемой переменной

        # Инициализация данных для графика
        self.x_data = []
        self.y_data = []

        # Настройка графика
        plt.ion()  # Включаем интерактивный режим
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-')  # Линия для траектории
        self.ax.set_xlabel('X position')
        self.ax.set_ylabel('Y position')
        self.ax.set_title('Robot Odometry Visualization')
        self.ax.grid(True)
        self.ax.set_xlim(-10, 10)  # Начальные пределы по X
        self.ax.set_ylim(-10, 10)  # Начальные пределы по Y

    def odom_callback(self, msg):
        # Извлекаем позицию робота
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Добавляем данные в списки
        self.x_data.append(x)
        self.y_data.append(y)

        # Обновляем график
        self.line.set_xdata(self.x_data)
        self.line.set_ydata(self.y_data)

        # Автоматически масштабируем оси
        self.ax.relim()
        self.ax.autoscale_view()

        # Перерисовываем график
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        # Выводим текущие координаты в консоль (опционально)
        self.get_logger().info(f'Position: x={x:.2f}, y={y:.2f}')

def main(args=None):
    rclpy.init(args=args)

    # Создаем ноду для визуализации
    odom_visualizer = OdomVisualizer()

    try:
        # Запускаем ноду
        rclpy.spin(odom_visualizer)
    except KeyboardInterrupt:
        # Останавливаем ноду при нажатии Ctrl+C
        pass
    finally:
        # Закрываем график и завершаем работу
        plt.close()
        odom_visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()