import tkinter as tk
from tkinter import ttk
import threading
import time
import re
import matplotlib
import math
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log

# Стандартные значения PID и скоростей
DEFAULT_VALUES = {
    'kp_left': 2.0,
    'ki_left': 0.0,
    'kd_left': 0.0,
    'kff_left': 2.25,
    'kp_right': 2.0,
    'ki_right': 0.0,
    'kd_right': 0.0,
    'kff_right': 2.25,
    'kp_speed_matching': 0.4,
    'target_left': 0,
    'target_right': 0
}

class ROSPublisher(Node):
    def __init__(self, callback):
        super().__init__('pid_tuner_publisher')
        
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.pid_pub = self.create_publisher(Float32MultiArray, 'params', qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.callback = callback
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.log_sub = self.create_subscription(
            Log,
            '/rosout',
            self.log_callback,
            10)
        self.get_logger().info("Subscribed to /rosout and cmd_vel")
        
        self.last_target_left = 0.0
        self.last_target_right = 0.0
        self.wheel_diameter = 0.034*2
        self.wheel_base = 0.16
        self.rpm_to_mps = (self.wheel_diameter * math.pi) / 60

    def cmd_vel_callback(self, msg):
        target_linear_speed = float(msg.linear.x)  # m/s
        target_angular_speed = float(msg.angular.z)  # rad/s
        
        # Вычисляем целевые скорости колес в RPM
        target_left_rpm = (target_linear_speed - target_angular_speed * self.wheel_base / 2.0) / self.rpm_to_mps
        target_right_rpm = (target_linear_speed + target_angular_speed * self.wheel_base / 2.0) / self.rpm_to_mps
        
        # Переводим в mm/s для отображения
        self.last_target_left = target_left_rpm * self.wheel_diameter * math.pi * 1000 / 60  # mm/s
        self.last_target_right = target_right_rpm * self.wheel_diameter * math.pi * 1000 / 60  # mm/s

    def log_callback(self, msg):
        if "left RPM" in msg.msg:
            pattern = r"left RPM: ([\d\.\-]+), right RPM: ([\d\.\-]+)"
            match = re.search(pattern, msg.msg)
            if match:
                left_rpm = float(match.group(1))
                right_rpm = float(match.group(2))
                
                left_speed = left_rpm * self.wheel_diameter * math.pi / 60 * 1000  # mm/s
                right_speed = right_rpm * self.wheel_diameter * math.pi / 60 * 1000  # mm/s
                
                target_left_rpm = self.last_target_left / (self.wheel_diameter * math.pi * 1000) * 60
                target_right_rpm = self.last_target_right / (self.wheel_diameter * math.pi * 1000) * 60
                
                self.callback({
                    'mm_s': f"SPD L={left_speed:.2f} mm/s R={right_speed:.2f} mm/s "
                           f"Target L={self.last_target_left:.2f} mm/s R={self.last_target_right:.2f} mm/s",
                    'rpm': f"SPD L={left_rpm:.2f} RPM R={right_rpm:.2f} RPM "
                          f"Target L={target_left_rpm:.2f} RPM R={target_right_rpm:.2f} RPM",
                    'left_rpm': left_rpm,
                    'right_rpm': right_rpm,
                    'left_speed': left_speed,
                    'right_speed': right_speed,
                    'target_left': self.last_target_left,
                    'target_right': self.last_target_right,
                    'target_left_rpm': target_left_rpm,
                    'target_right_rpm': target_right_rpm
                })

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("PID Tuner and Speed Monitor")
        
        self.unit_var = tk.StringVar(value="mm/s")
        self.create_widgets()
        
        rclpy.init()
        self.ros_node = ROSPublisher(self.handle_speed_data)
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()
        
        self.figure = Figure(figsize=(12,4), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("Wheel Speed Response")
        self.ax.set_xlabel("Time (s)")
        
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.root)
        self.canvas.get_tk_widget().grid(row=10, column=0, columnspan=5, pady=10)
        
        self.time_data = []
        self.measured_left_data = []
        self.target_left_data = []
        self.measured_right_data = []
        self.target_right_data = []
        self.measured_left_rpm_data = []
        self.target_left_rpm_data = []
        self.measured_right_rpm_data = []
        self.target_right_rpm_data = []
        self.start_time = time.time()
        
        self.last_target_left = 0.0
        self.last_target_right = 0.0
        
        self.update_plot()

    def create_widgets(self):
        tk.Label(self.root, text="Left Wheel PID:").grid(row=0, column=0, columnspan=2)
        tk.Label(self.root, text="Kp:").grid(row=1, column=0, sticky="e")
        self.kp_left_scale = tk.Scale(self.root, from_=0, to=20, resolution=0.1,
                                     orient=tk.HORIZONTAL, length=200,
                                     command=self.update_coefficients)
        self.kp_left_scale.set(DEFAULT_VALUES['kp_left'])
        self.kp_left_scale.grid(row=1, column=1)
        
        tk.Label(self.root, text="Ki:").grid(row=1, column=2, sticky="e")
        self.ki_left_scale = tk.Scale(self.root, from_=0, to=20, resolution=0.1,
                                     orient=tk.HORIZONTAL, length=200,
                                     command=self.update_coefficients)
        self.ki_left_scale.set(DEFAULT_VALUES['ki_left'])
        self.ki_left_scale.grid(row=1, column=3)
        
        tk.Label(self.root, text="Kd:").grid(row=2, column=0, sticky="e")
        self.kd_left_scale = tk.Scale(self.root, from_=0, to=5, resolution=0.01,
                                     orient=tk.HORIZONTAL, length=200,
                                     command=self.update_coefficients)
        self.kd_left_scale.set(DEFAULT_VALUES['kd_left'])
        self.kd_left_scale.grid(row=2, column=1)
        
        tk.Label(self.root, text="Kff:").grid(row=2, column=2, sticky="e")
        self.kff_left_scale = tk.Scale(self.root, from_=0, to=10, resolution=0.05,
                                      orient=tk.HORIZONTAL, length=200,
                                      command=self.update_coefficients)
        self.kff_left_scale.set(DEFAULT_VALUES['kff_left'])
        self.kff_left_scale.grid(row=2, column=3)

        tk.Label(self.root, text="Right Wheel PID:").grid(row=3, column=0, columnspan=2)
        tk.Label(self.root, text="Kp:").grid(row=4, column=0, sticky="e")
        self.kp_right_scale = tk.Scale(self.root, from_=0, to=20, resolution=0.1,
                                      orient=tk.HORIZONTAL, length=200,
                                      command=self.update_coefficients)
        self.kp_right_scale.set(DEFAULT_VALUES['kp_right'])
        self.kp_right_scale.grid(row=4, column=1)
        
        tk.Label(self.root, text="Ki:").grid(row=4, column=2, sticky="e")
        self.ki_right_scale = tk.Scale(self.root, from_=0, to=20, resolution=0.1,
                                      orient=tk.HORIZONTAL, length=200,
                                      command=self.update_coefficients)
        self.ki_right_scale.set(DEFAULT_VALUES['ki_right'])
        self.ki_right_scale.grid(row=4, column=3)
        
        tk.Label(self.root, text="Kd:").grid(row=5, column=0, sticky="e")
        self.kd_right_scale = tk.Scale(self.root, from_=0, to=5, resolution=0.01,
                                      orient=tk.HORIZONTAL, length=200,
                                      command=self.update_coefficients)
        self.kd_right_scale.set(DEFAULT_VALUES['kd_right'])
        self.kd_right_scale.grid(row=5, column=1)
        
        tk.Label(self.root, text="Kff:").grid(row=5, column=2, sticky="e")
        self.kff_right_scale = tk.Scale(self.root, from_=0, to=10, resolution=0.05,
                                       orient=tk.HORIZONTAL, length=200,
                                       command=self.update_coefficients)
        self.kff_right_scale.set(DEFAULT_VALUES['kff_right'])
        self.kff_right_scale.grid(row=5, column=3)

        tk.Label(self.root, text="Speed Matching Kp:").grid(row=6, column=0, columnspan=2, sticky="e")
        self.kp_speed_matching_scale = tk.Scale(self.root, from_=0, to=10, resolution=0.1,
                                               orient=tk.HORIZONTAL, length=200,
                                               command=self.update_coefficients)
        self.kp_speed_matching_scale.set(DEFAULT_VALUES['kp_speed_matching'])
        self.kp_speed_matching_scale.grid(row=6, column=2, columnspan=2)

        tk.Label(self.root, text="Target Speed Left (mm/s):").grid(row=7, column=0, columnspan=2)
        self.target_left_scale = tk.Scale(self.root, from_=-2000, to=2000, resolution=1,
                                         orient=tk.HORIZONTAL, length=200,
                                         command=self.update_speed)
        self.target_left_scale.set(DEFAULT_VALUES['target_left'])
        self.target_left_scale.grid(row=7, column=2)
        
        tk.Label(self.root, text="Target Speed Right (mm/s):").grid(row=7, column=3)
        self.target_right_scale = tk.Scale(self.root, from_=-2000, to=2000, resolution=1,
                                          orient=tk.HORIZONTAL, length=200,
                                          command=self.update_speed)
        self.target_right_scale.set(DEFAULT_VALUES['target_right'])
        self.target_right_scale.grid(row=7, column=4)

        tk.Label(self.root, text="Units:").grid(row=8, column=0, sticky="e")
        unit_options = ttk.OptionMenu(self.root, self.unit_var, "mm/s", "mm/s", "RPM", 
                                    command=self.update_plot_units)
        unit_options.grid(row=8, column=1)

        self.reset_defaults_button = tk.Button(self.root, text="Reset to Defaults", 
                                             command=self.reset_to_defaults)
        self.reset_defaults_button.grid(row=8, column=3, pady=5, sticky="e")

        self.status_text = tk.Text(self.root, height=5, width=100)
        self.status_text.grid(row=11, column=0, columnspan=5, pady=5)

    def spin_ros(self):
        while rclpy.ok():
            rclpy.spin_once(self.ros_node)
            time.sleep(0.01)

    def handle_speed_data(self, data):
        self.status_text.insert(tk.END, data['mm_s'] + "\n")
        self.status_text.insert(tk.END, data['rpm'] + "\n")
        self.status_text.see(tk.END)
        
        try:
            current_time = time.time() - self.start_time
            self.time_data.append(current_time)
            self.measured_left_data.append(data['left_speed'])
            self.target_left_data.append(data['target_left'])
            self.measured_right_data.append(data['right_speed'])
            self.target_right_data.append(data['target_right'])
            self.measured_left_rpm_data.append(data['left_rpm'])
            self.target_left_rpm_data.append(data['target_left_rpm'])
            self.measured_right_rpm_data.append(data['right_rpm'])
            self.target_right_rpm_data.append(data['target_right_rpm'])
        except Exception as e:
            print("Error parsing speed data:", e)

    def update_coefficients(self, value):
        pid_msg = Float32MultiArray()
        pid_msg.data = [
            float(self.kp_left_scale.get()), float(self.ki_left_scale.get()), 
            float(self.kd_left_scale.get()), float(self.kff_left_scale.get()),
            float(self.kp_right_scale.get()), float(self.ki_right_scale.get()),
            float(self.kd_right_scale.get()), float(self.kff_right_scale.get()),
            float(self.kp_speed_matching_scale.get())
        ]
        self.ros_node.pid_pub.publish(pid_msg)
        self.status_text.insert(tk.END, f"Sent PID: {pid_msg.data}\n")
        self.status_text.see(tk.END)

    def update_speed(self, value):
        twist = Twist()
        left_speed = float(self.target_left_scale.get()) / 1000.0  # mm/s to m/s
        right_speed = float(self.target_right_scale.get()) / 1000.0  # mm/s to m/s
        
        wheel_base = 0.16
        twist.linear.x = (left_speed + right_speed) / 2
        twist.angular.z = (right_speed - left_speed) / wheel_base
        
        self.last_target_left = float(self.target_left_scale.get())
        self.last_target_right = float(self.target_right_scale.get())
        self.ros_node.last_target_left = self.last_target_left
        self.ros_node.last_target_right = self.last_target_right
        
        self.ros_node.cmd_vel_pub.publish(twist)
        self.status_text.insert(tk.END, f"Sent cmd_vel: linear.x={twist.linear.x}, angular.z={twist.angular.z}\n")
        self.status_text.see(tk.END)

    def reset_all(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.ros_node.cmd_vel_pub.publish(twist)
        self.target_left_scale.set(0)
        self.target_right_scale.set(0)
        self.last_target_left = 0.0
        self.last_target_right = 0.0
        self.ros_node.last_target_left = 0.0
        self.ros_node.last_target_right = 0.0
        self.status_text.insert(tk.END, "Reset speeds to 0\n")

        pid_msg = Float32MultiArray()
        pid_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.ros_node.pid_pub.publish(pid_msg)
        self.kp_left_scale.set(0)
        self.ki_left_scale.set(0)
        self.kd_left_scale.set(0)
        self.kff_left_scale.set(0)
        self.kp_right_scale.set(0)
        self.ki_right_scale.set(0)
        self.kd_right_scale.set(0)
        self.kff_right_scale.set(0)
        self.kp_speed_matching_scale.set(0)
        self.status_text.insert(tk.END, f"Reset PID: {pid_msg.data}\n")
        self.status_text.see(tk.END)

    def reset_to_defaults(self):
        self.kp_left_scale.set(DEFAULT_VALUES['kp_left'])
        self.ki_left_scale.set(DEFAULT_VALUES['ki_left'])
        self.kd_left_scale.set(DEFAULT_VALUES['kd_left'])
        self.kff_left_scale.set(DEFAULT_VALUES['kff_left'])
        self.kp_right_scale.set(DEFAULT_VALUES['kp_right'])
        self.ki_right_scale.set(DEFAULT_VALUES['ki_right'])
        self.kd_right_scale.set(DEFAULT_VALUES['kd_right'])
        self.kff_right_scale.set(DEFAULT_VALUES['kff_right'])
        self.kp_speed_matching_scale.set(DEFAULT_VALUES['kp_speed_matching'])
        self.target_left_scale.set(DEFAULT_VALUES['target_left'])
        self.target_right_scale.set(DEFAULT_VALUES['target_right'])
        
        self.last_target_left = float(DEFAULT_VALUES['target_left'])
        self.last_target_right = float(DEFAULT_VALUES['target_right'])
        self.ros_node.last_target_left = self.last_target_left
        self.ros_node.last_target_right = self.last_target_right
        
        pid_msg = Float32MultiArray()
        pid_msg.data = [
            float(DEFAULT_VALUES['kp_left']), float(DEFAULT_VALUES['ki_left']),
            float(DEFAULT_VALUES['kd_left']), float(DEFAULT_VALUES['kff_left']),
            float(DEFAULT_VALUES['kp_right']), float(DEFAULT_VALUES['ki_right']),
            float(DEFAULT_VALUES['kd_right']), float(DEFAULT_VALUES['kff_right']),
            float(DEFAULT_VALUES['kp_speed_matching'])
        ]
        self.ros_node.pid_pub.publish(pid_msg)
        
        twist = Twist()
        left_speed = float(DEFAULT_VALUES['target_left']) / 1000.0
        right_speed = float(DEFAULT_VALUES['target_right']) / 1000.0
        wheel_base = 0.16
        twist.linear.x = (left_speed + right_speed) / 2
        twist.angular.z = (right_speed - left_speed) / wheel_base
        self.ros_node.cmd_vel_pub.publish(twist)
        
        self.status_text.insert(tk.END, "Reset to default values\n")
        self.status_text.insert(tk.END, f"PID: {pid_msg.data}\n")
        self.status_text.insert(tk.END, f"Speeds: linear.x={twist.linear.x}, angular.z={twist.angular.z}\n")
        self.status_text.see(tk.END)

    def update_plot_units(self, value):
        self.update_plot()

    def update_plot(self, *args):
        self.ax.clear()
        self.ax.set_title("Wheel Speed Response")
        self.ax.set_xlabel("Time (s)")
        
        unit = self.unit_var.get()
        if unit == "mm/s":
            self.ax.set_ylabel("Speed (mm/s)")
            self.ax.plot(self.time_data, self.measured_left_data, label="Measured Left")
            self.ax.plot(self.time_data, self.target_left_data, label="Target Left", linestyle="--")
            self.ax.plot(self.time_data, self.measured_right_data, label="Measured Right")
            self.ax.plot(self.time_data, self.target_right_data, label="Target Right", linestyle="--")
        else:  # RPM
            self.ax.set_ylabel("Speed (RPM)")
            self.ax.plot(self.time_data, self.measured_left_rpm_data, label="Measured Left")
            self.ax.plot(self.time_data, self.target_left_rpm_data, label="Target Left", linestyle="--")
            self.ax.plot(self.time_data, self.measured_right_rpm_data, label="Measured Right")
            self.ax.plot(self.time_data, self.target_right_rpm_data, label="Target Right", linestyle="--")
        
        self.ax.legend()
        
        if self.time_data:
            current_time = self.time_data[-1]
            if current_time < 20:
                self.ax.set_xlim(0, 20)
            else:
                self.ax.set_xlim(current_time - 20, current_time)
        
        self.canvas.draw()
        self.root.after(1000, self.update_plot)

    def on_closing(self):
        def shutdown():
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.ros_node.cmd_vel_pub.publish(twist)
            self.status_text.insert(tk.END, "Stopping motors before shutdown\n")
            self.status_text.see(tk.END)
            time.sleep(1.0)
            
            rclpy.shutdown()
            self.root.quit()

        shutdown_thread = threading.Thread(target=shutdown)
        shutdown_thread.start()

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()