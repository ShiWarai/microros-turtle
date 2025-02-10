#include "microros/microros.hpp"


// Распиновка
constexpr uint8_t ENCODER_M1_A = 27;
constexpr uint8_t ENCODER_M1_B = 26;

constexpr uint8_t EN_M1 = 13;
constexpr uint8_t IN1_M1 = 12;
constexpr uint8_t IN2_M1 = 14;

constexpr uint8_t ENCODER_M2_B = 25;
constexpr uint8_t ENCODER_M2_A = 33;

constexpr uint8_t EN_M2 = 18;
constexpr uint8_t IN1_M2 = 21;
constexpr uint8_t IN2_M2 = 19;

// Физические параметры робота
constexpr float WHEEL_BASE = 0.3f;	  // Расстояние между колесами (метры)
constexpr float WHEEL_RADIUS = 0.07f; // Радиус колес (метры)
constexpr uint16_t FULL_ROTATE = 374;
constexpr float RPM_TO_MPS = 2 * PI * WHEEL_RADIUS / 60.0f; // Константы для пересчёта RPM в скорость

DECLARE_ENCODER_WITH_NAME(LEFT, ENCODER_M1_A, ENCODER_M1_B)
DECLARE_ENCODER_WITH_NAME(RIGHT, ENCODER_M2_A, ENCODER_M2_B)

// Инициализация microROS
rcl_publisher_t publisher;
rcl_subscription_t subscriber;

L298N driverA(EN_M1, IN1_M1, IN2_M1);
L298N driverB(EN_M2, IN1_M2, IN2_M2);
MotorController motorA(driverA, encoder_LEFT, FULL_ROTATE);
MotorController motorB(driverB, encoder_RIGHT, FULL_ROTATE);
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_subscription_t params_subscriber;
std_msgs__msg__Float32MultiArray params_msg;

rcl_publisher_t odom_publisher;
rcl_timer_t odom_timer;
nav_msgs__msg__Odometry odom_msg;

DreameLidar *lidar;
rcl_publisher_t lidar_publisher;
rcl_timer_t lidar_timer;
sensor_msgs__msg__LaserScan lidar_msg;

MPU9250_DMP imu;
rcl_publisher_t imu_publisher;
rcl_timer_t imu_timer;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t log_publisher;
std_msgs__msg__String log_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Новый объект для хранения текущей позиции для одометрии
float x = 0.0f;		// Позиция по X (м)
float y = 0.0f;		// Позиция по Y (м)
float theta = 0.0f; // Ориентация (рад)

// Ориентация по умолчанию (в 3D)
const signed char orientationDefault[9] = {0, 1, 0, 0, 0, 1, 1, 0, 0};

// volatile bool interrupt = false;

IPAddress getIPAddressByHostname(const char *hostname)
{
	IPAddress remote_ip;

	if (WiFi.hostByName(hostname, remote_ip))
	{
		return remote_ip;
	}
	else
	{
		Serial.println("Failed to resolve hostname");
		return IPAddress(0, 0, 0, 0);
	}
}

void error_loop()
{
	while (1)
	{
		Serial.println("Error!");
		vTaskDelay(100);
	}
}

void cmd_vel_callback(const void *msgin)
{
	const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

	float linear_speed = float(msg->linear.x);	 // Линейная скорость (м/с)
	float angular_speed = float(msg->angular.z); // Угловая скорость (рад/с)

	// Вычисляем целевые скорости колес
	float left_wheel_rpm = (linear_speed - angular_speed * WHEEL_BASE / 2.0) * (60 / (2 * PI * WHEEL_RADIUS));
	float right_wheel_rpm = (linear_speed + angular_speed * WHEEL_BASE / 2.0) * (60 / (2 * PI * WHEEL_RADIUS));

	motorA.setTargetRPM(left_wheel_rpm);
	motorB.setTargetRPM(right_wheel_rpm);

	// // Для отладки выводим значения
	// Serial.print("Linear: ");
	// Serial.print(linear_speed);
	// Serial.print(" Angular: ");
	// Serial.println(angular_speed);
	// Serial.print("Left RPM: ");
	// Serial.print(left_wheel_rpm);
	// Serial.print(" Right RPM: ");
	// Serial.println(right_wheel_rpm);
}

void params_callback(const void *msgin)
{
	const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;

	if (msg->data.size == 6)
	{
		motorA.setPIDConfig(msg->data.data[0], msg->data.data[1], msg->data.data[2]);
		motorB.setPIDConfig(msg->data.data[3], msg->data.data[4], msg->data.data[5]);
	}
	else
		Serial.println("Invalid PID parameters received!");
}

// Преобразование угла в кватернион
void set_orientation(geometry_msgs__msg__Quaternion &orientation, double theta)
{
	// Вычисляем компоненты кватерниона для поворота вокруг оси Z (в 2D)
	double qx = 0.0;
	double qy = 0.0;
	double qz = sin(theta / 2.0);
	double qw = cos(theta / 2.0);

	// Устанавливаем значения в сообщение
	orientation.x = qx;
	orientation.y = qy;
	orientation.z = qz;
	orientation.w = qw;
}

void update_odometry(float left_rpm, float right_rpm, float dt)
{
	// Конвертируем RPM в линейные скорости
	float left_velocity = RPM_TO_MPS * left_rpm;   // Левое колесо (м/с)
	float right_velocity = RPM_TO_MPS * right_rpm; // Правое колесо (м/с)

	// Вычисляем линейную и угловую скорость робота
	float linear_velocity = (left_velocity + right_velocity) / 2.0f;
	float angular_velocity = (right_velocity - left_velocity) / WHEEL_BASE;

	// Интегрируем для вычисления новой позиции
	float delta_theta = angular_velocity * dt;
	theta += delta_theta;
	theta = fmod(theta + 2 * PI, 2 * PI); // Нормализация угла

	float delta_x = linear_velocity * cos(theta) * dt;
	float delta_y = linear_velocity * sin(theta) * dt;

	x += delta_x;
	y += delta_y;

	// Обновление одометрического сообщения
	odom_msg.header.stamp.nanosec = esp_timer_get_time() * 1000;

	// Позиция
	odom_msg.pose.pose.position.x = x;
	odom_msg.pose.pose.position.y = y;
	odom_msg.pose.pose.position.z = 0.0f;

	// Ориентация (в формате кватерниона)
	set_orientation(odom_msg.pose.pose.orientation, theta);

	// Скорости
	odom_msg.twist.twist.linear.x = linear_velocity;
	odom_msg.twist.twist.linear.y = 0.0f; // В нашем случае скорости в Y нет
	odom_msg.twist.twist.angular.z = angular_velocity;

	// Serial.printf("Odometry update: Position (x: %.2f, y: %.2f, theta: %.2f), Linear Velocity: %.2f, Angular Velocity: %.2f\r\n",
	//             x, y, theta, linear_velocity, angular_velocity);
}

void log_to_topic(const char* log_message) {
    // Формируем сообщение
    snprintf(log_msg.data.data, log_msg.data.capacity, "%s", log_message);
    log_msg.data.size = strlen(log_msg.data.data);

    // Публикуем сообщение в топик
    RCSOFTCHECK(rcl_publish(&log_publisher, &log_msg, NULL));
}

void odometry_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	motorA.update();
	motorB.update();

	if (timer != NULL)
	{
		// Получение данных с моторов
		float left_rpm = motorA.getCurrentRPM();
		float right_rpm = motorB.getCurrentRPM();

		// Расчёт времени между вызовами таймера (dt)
		static int64_t last_time = 0;
		int64_t current_time = esp_timer_get_time();
		float dt = (current_time - last_time) / 1e6f; // В секундах
		last_time = current_time;

		// Обновление одометрии
		update_odometry(left_rpm, right_rpm, dt);

		// Публикация данных одометрии
		RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
	}
}

void imu_update()
{
	unsigned short fifoCnt;
	inv_error_t result;

	
	fifoCnt = imu.fifoAvailable();

	if (fifoCnt > 0)
	{
		result = imu.dmpUpdateFifo();

		if (result == INV_SUCCESS)
		{

			float q0 = imu.calcQuat(imu.qw);
			float q1 = imu.calcQuat(imu.qx);
			float q2 = imu.calcQuat(imu.qy);
			float q3 = imu.calcQuat(imu.qz);

			imu_msg.angular_velocity.x = imu.calcGyro(imu.gx) * PI / 180.0;
			imu_msg.angular_velocity.y = imu.calcGyro(imu.gy) * PI / 180.0;
			imu_msg.angular_velocity.z = imu.calcGyro(imu.gz) * PI / 180.0;

			imu_msg.linear_acceleration.x = imu.calcAccel(imu.ax) * 10;
			imu_msg.linear_acceleration.y = imu.calcAccel(imu.ay) * 10;
			imu_msg.linear_acceleration.z = imu.calcAccel(imu.az) * 10;

			//imu.computeEulerAngles();
			// Serial.printf("%f, %f, %f\r\n", imu.roll * 180 / PI, imu.pitch * 180 / PI, imu.yaw * 180 / PI);

			imu_msg.orientation.w = q0;
			imu_msg.orientation.x = q1;
			imu_msg.orientation.y = q2;
			imu_msg.orientation.z = q3;

			imu_msg.header.stamp.nanosec = esp_timer_get_time() * 1000;

			RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
		}
	}
}

void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	imu_update();
}

void lidar_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	if (lidar->dataObtained)
	{
		if (xSemaphoreTake(lidar->dataLock, portMAX_DELAY) == pdTRUE)
		{
			for(size_t i = 0; i < lidar_msg.ranges.capacity; i++)
				lidar_msg.ranges.data[i] = 0;

			int deg;
			lidar_msg.header.stamp.nanosec = esp_timer_get_time() * 1000;

			for (int i = 0; i < lidar->dataSize; i++)
			{
				deg = round(-lidar->theta[i] * 180.0 / M_PI);
				if (lidar->theta[i] < lidar_msg.angle_max && lidar->theta[i] > lidar_msg.angle_min)
				{
					lidar_msg.ranges.data[deg] = ((float)lidar->distance[i]) / 1000.0;
					#ifdef LIDAR_INTENSITY
					lidar_msg.intensities.data[deg] = lidar->intensity[i];
					#endif
				}
			}

			lidar->dataObtained = false; // Сбрасываем флаг после вывода данных
			xSemaphoreGive(lidar->dataLock);
			
			RCSOFTCHECK(rcl_publish(&lidar_publisher, &lidar_msg, NULL));
		}
	}
}

void init_msgs_params()
{
	params_msg.data.capacity = 6;
	params_msg.data.size = 6;
	params_msg.data.data = new float[params_msg.data.capacity];
}

void init_msgs_odometry()
{
	odom_msg.header.frame_id.capacity = 12;
	odom_msg.header.frame_id.size = 11;
	odom_msg.header.frame_id.data = new char[odom_msg.header.frame_id.capacity];
	odom_msg.header.frame_id = micro_ros_string_utilities_init("odom");

	// odom_msg.child_frame_id.capacity = 8;
	// odom_msg.child_frame_id.size = 7;
	// odom_msg.child_frame_id.data = new char[odom_msg.child_frame_id.capacity];
	// odom_msg.child_frame_id = micro_ros_string_utilities_init("base_link");

	odom_msg.pose.covariance[0] = 0.1; // Примерные значения ковариации
	odom_msg.pose.covariance[7] = 0.1;
	odom_msg.pose.covariance[14] = 1e-3;
	odom_msg.pose.covariance[21] = 1e-3;
	odom_msg.pose.covariance[28] = 1e-3;
	odom_msg.pose.covariance[35] = 0.1;

	odom_msg.twist.covariance[0] = 0.1;
	odom_msg.twist.covariance[7] = 0.1;
	odom_msg.twist.covariance[14] = 1e-3;
	odom_msg.twist.covariance[21] = 1e-3;
	odom_msg.twist.covariance[28] = 1e-3;
	odom_msg.twist.covariance[35] = 0.1;
}

void init_msgs_imu()
{
	imu_msg.header.frame_id.capacity = 12;
	imu_msg.header.frame_id.size = 11;
	imu_msg.header.frame_id.data = new char[imu_msg.header.frame_id.capacity];
	imu_msg.header.frame_id = micro_ros_string_utilities_init("imu_link");
}

void init_msgs_laserscan()
{
	lidar_msg.header.frame_id.capacity = 12;
	lidar_msg.header.frame_id.size = 11;
	lidar_msg.header.frame_id.data = new char[lidar_msg.header.frame_id.capacity];

	lidar_msg.header.frame_id = micro_ros_string_utilities_init("laser_frame");

	lidar_msg.ranges.capacity = LIDAR_DATA_SIZE;
	lidar_msg.ranges.size = LIDAR_DATA_SIZE;
	lidar_msg.ranges.data = new float[lidar_msg.ranges.capacity];

	#ifdef LIDAR_INTENSITY
// lidar_msg.intensities.capacity = lidar_msg.ranges.capacity;
// lidar_msg.intensities.size = lidar_msg.intensities.capacity;
// lidar_msg.intensities.data = new float[lidar_msg.intensities.capacity];
	#endif

	lidar_msg.angle_min = -2 * M_PI;
	lidar_msg.angle_max = 0;
	lidar_msg.angle_increment = 2 * M_PI / lidar_msg.ranges.capacity;
	lidar_msg.range_min = 0.01;
	lidar_msg.range_max = 10.0;
}

void MicroRosController::microrosTask(void *pvParameters)
{
	IPAddress agent_ip_address = IPAddress();
	agent_ip_address.fromString(settings.agent_ip);

#ifdef AGENT_IP
	static struct micro_ros_agent_locator locator;
	locator.address = agent_ip_address;
	locator.port = settings.agent_port;

	rmw_uros_set_custom_transport(
		false,
		(void *)&locator,
		platformio_transport_open,
		platformio_transport_close,
		platformio_transport_write,
		platformio_transport_read);
#else
#error "NEED FIX THIS BLOCK"
	set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, getIPAddressByHostname(AGENT_HOSTNAME), AGENT_PORT);
#endif

	allocator = rcl_get_default_allocator();

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	RCCHECK(rclc_node_init_default(&node, String("body_node_tutle_" + settings.turtle_id).c_str(), "", &support));

	RCCHECK(rclc_subscription_init_best_effort(
		&cmd_vel_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"/cmd_vel"));

	RCCHECK(rclc_subscription_init_best_effort(
		&params_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
		"/params"));

	RCCHECK(rclc_publisher_init_default(
		&odom_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
		"/odometry"));

	RCCHECK(rclc_timer_init_default(
		&odom_timer,
		&support,
		RCL_MS_TO_NS(settings.odom_delay),
		odometry_timer_callback));

	RCCHECK(rclc_publisher_init_default(
		&imu_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"/imu"));

	RCCHECK(rclc_timer_init_default(
		&imu_timer,
		&support,
		RCL_MS_TO_NS(settings.imu_delay),
		imu_timer_callback));

	RCCHECK(rclc_publisher_init_default(
		&lidar_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
		"/lidar"));

	RCCHECK(rclc_timer_init_default(
		&lidar_timer,
		&support,
		RCL_MS_TO_NS(settings.lidar_delay),
		lidar_timer_callback));

	RCCHECK(rclc_publisher_init_default(
		&log_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
		"/logs"));

	init_msgs_laserscan();
	init_msgs_imu();
	init_msgs_odometry();
	init_msgs_params();

	RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator)); // create executor
	RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &params_subscriber, &params_msg, &params_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
	RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
	RCCHECK(rclc_executor_add_timer(&executor, &lidar_timer));

	//rcutils_logging_set_output_handler(rcutils_logging_output_handler);
  	//rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_INFO);

	lidar = new DreameLidar(&Serial2);
	xTaskCreate(DreameLidar::lidarTask, "lidar_data_task", 8196, lidar, 1, NULL);

	INIT_ENCODER_WITH_NAME(LEFT, ENCODER_M1_A, ENCODER_M1_B)
	INIT_ENCODER_WITH_NAME(RIGHT, ENCODER_M2_A, ENCODER_M2_B)

	motorA.setPIDConfig(20.0, 0.03, 500.0);
	motorB.setPIDConfig(20.0, 0.03, 500.0);

	Wire.begin(22, 23); // 22 - SDA, 23 - SCL
	if (imu.begin() != INV_SUCCESS)
	{
		while (true)
		{
			Serial.println("Unable to communicate with MPU-9250");
			Serial.println("Check connections, and try again.");
			Serial.println();
			delay(1000);
		}
	}

	imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	imu.setAccelFSR(2);
	imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT |
					DMP_FEATURE_GYRO_CAL |
					DMP_FEATURE_SEND_CAL_GYRO |
					DMP_FEATURE_SEND_RAW_ACCEL,
					10);
	imu.dmpSetOrientation(orientationDefault);

	// Инициализация сообщения
    log_msg.data.data = (char *)malloc(256 * sizeof(char));
    log_msg.data.capacity = 256;
    log_msg.data.size = 0;

	Serial.println("ROS started!");
	while (true)
	{
		RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
		vTaskDelay(1);
	}
}