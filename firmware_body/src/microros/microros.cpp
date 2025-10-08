#include "microros/microros.hpp"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ Serial.printf("Error: %d\r\n", temp_rc); error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) {}}

// Физические параметры робота
constexpr float RPM_TO_MPS = 2 * PI * WHEEL_RADIUS / 60.0f; // Константы для пересчёта RPM в скорость
constexpr float RAD_TO_ITER = RAD_TO_DEG * (RANGES_SIZE / 360.0); // Константа для лидара

DECLARE_ENCODER_WITH_NAME(LEFT, ENCODER_M1_A, ENCODER_M1_B)
DECLARE_ENCODER_WITH_NAME(RIGHT, ENCODER_M2_A, ENCODER_M2_B)

SpeedMatchingRegulator speedMatchingController(0.4f);

// Инициализация microROS
rcl_publisher_t publisher;
rcl_subscription_t subscriber;

L298N driverA(EN_M1, IN1_M1, IN2_M1);
L298N driverB(EN_M2, IN1_M2, IN2_M2);
MotorController motorA(driverA, encoder_LEFT, FULL_ROTATE);
MotorController motorB(driverB, encoder_RIGHT, FULL_ROTATE);
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_subscription_t pid_params_subscriber;
std_msgs__msg__Float32MultiArray pid_params_msg;

rcl_publisher_t odom_publisher;
rcl_timer_t odom_timer;
nav_msgs__msg__Odometry odom_msg;

DreameLidar *lidar;
rcl_publisher_t lidar_publisher;
rcl_timer_t lidar_timer;
sensor_msgs__msg__LaserScan lidar_msg;

#ifdef MPU_9250
MPU9250_DMP imu;
#endif
#ifdef MPU_6050
MPU6050 mpu;
uint8_t fifoBuffer[45];         // буфер
#endif
rcl_publisher_t imu_publisher;
rcl_timer_t imu_timer;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t log_publisher;
rcl_timer_t log_timer;
rcl_interfaces__msg__Log log_msg;

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

float normalize_angle(float angle) {
    angle = fmod(angle, 2.0 * M_PI); // Приводим угол к диапазону от -2π до 2π
    if (angle < 0) {
        angle += 2.0 * M_PI; // Приводим угол к диапазону от 0 до 2π
    }
    return angle;
}

void set_timestamp(builtin_interfaces__msg__Time* stamp)
{
    if (rmw_uros_epoch_synchronized()) { // Проверяем, синхронизировано ли время
        // Получаем время в наносекундах
        int64_t time_ns = rmw_uros_epoch_nanos();
        stamp->sec = time_ns / 1000000000LL;       // Секунды
        stamp->nanosec = time_ns % 1000000000LL;   // Наносекунды
    } else {
        // Fallback, если синхронизация ещё не выполнена
        int64_t current_time = esp_timer_get_time(); // В микросекундах
        stamp->sec = current_time / 1000000;
        stamp->nanosec = (current_time % 1000000) * 1000;
    }
}

void error_loop()
{
	while (1)
	{
		Serial.println("Error!");
		vTaskDelay(5000);
	}
}

void cmd_vel_callback(const void *msgin)
{
	const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *) msgin;

    float target_linear_speed = float(msg->linear.x);   // Линейная скорость (м/с)
    float target_angular_speed = float(msg->angular.z); // Угловая скорость (рад/с)

    // Вычисляем целевые скорости колес
    float target_left_rpm = (target_linear_speed - target_angular_speed * WHEEL_BASE / 2.0) / RPM_TO_MPS;
    float target_right_rpm = (target_linear_speed + target_angular_speed * WHEEL_BASE / 2.0) / RPM_TO_MPS;

    motorA.setTargetRPM(target_left_rpm);
    motorB.setTargetRPM(target_right_rpm);

	char str[256];
	sprintf(str, "target linear: %.2f, target angular: %.2f, target left RPM: %.2f, target right RPM: %.2f", target_linear_speed, target_angular_speed, target_left_rpm, target_right_rpm);
	MicroROSLogger::log(str, "cmd_vel_callback()", "microros.cpp", LogLevel::INFO, false);
}

void pid_params_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;

	char str[128];
    if (msg->data.size == 9)
    {
        motorA.setPIDConfig(msg->data.data[0], msg->data.data[1], msg->data.data[2], msg->data.data[3]);
        motorB.setPIDConfig(msg->data.data[4], msg->data.data[5], msg->data.data[6], msg->data.data[7]);
		speedMatchingController.setKp(msg->data.data[8]);

        sprintf(str, "Pid update: MotorA [Kp=%.2f, Ki=%.2f, Kd=%.2f, Kff=%.2f], MotorB [Kp=%.2f, Ki=%.2f, Kd=%.2f, Kff=%.2f]",
                msg->data.data[0], msg->data.data[1], msg->data.data[2], msg->data.data[3],
                msg->data.data[4], msg->data.data[5], msg->data.data[6], msg->data.data[7]);
        MicroROSLogger::log(str, "pid_params_callback()", "microros.cpp", LogLevel::INFO, false);
    }
    else
    {
        sprintf(str, "Invalid PID parameters received: size=%zu", msg->data.size);
        MicroROSLogger::log(str, "pid_params_callback()", "microros.cpp", LogLevel::WARN, true);
    }
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

volatile long last_time = 0;
void odometry_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	long current_time = millis();
	float dt = current_time - last_time;
	last_time = current_time;
	
	// Получение данных с моторов
	float left_rpm = motorA.getCurrentRPM();
	float right_rpm = motorB.getCurrentRPM();

    // Корректировка скоростей при движении по прямой
    float correction = speedMatchingController.compute(left_rpm, right_rpm, motorA.getTargetRPM(), motorB.getTargetRPM());

	// Передача корректировки и шага для обновления регулятора
	motorA.update(dt, -correction);
	motorB.update(dt, correction);

	if (timer != NULL)
	{
		// Конвертируем RPM в линейные скорости
		float left_velocity = RPM_TO_MPS * left_rpm;   // Левое колесо (м/с)
		float right_velocity = RPM_TO_MPS * right_rpm; // Правое колесо (м/с)

		// Вычисляем линейную и угловую скорость робота
		float linear_velocity = (left_velocity + right_velocity) / 2.0f;
		float angular_velocity = (right_velocity - left_velocity) / WHEEL_BASE;

		// Интегрируем для вычисления новой позиции
		float delta_theta = angular_velocity * (dt / 1e3);
		theta += delta_theta;
		theta = fmod(theta + 2 * PI, 2 * PI); // Нормализация угла

		float delta_x = linear_velocity * cos(theta) * (dt / 1e3);
		float delta_y = linear_velocity * sin(theta) * (dt / 1e3);

		x += delta_x;
		y += delta_y;

		// Обновление одометрического сообщения
		set_timestamp(&odom_msg.header.stamp);

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

		char str[256];
		sprintf(str, "left RPM: %.2f, right RPM: %.2f, position (x: %.2f, y: %.2f, theta: %.2f), linear velocity: %.2f, angular velocity: %.2f",
					left_rpm, right_rpm, x, y, theta, linear_velocity, angular_velocity);
		MicroROSLogger::log(str, "odometry_timer_callback()", "microros.cpp", LogLevel::INFO, false);

		RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
	}
}

void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	#ifdef MPU_9250
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

			// char str[128];
			// imu.computeEulerAngles();
			// sprintf(str, "roll: %.2f, pitch: %.2f, yaw: %.2f", imu.roll * 180 / PI, imu.pitch * 180 / PI, imu.yaw * 180 / PI);
			// MicroROSLogger::log(str, "odometry_timer_callback()", "microros.cpp", LogLevel::INFO, false);

			imu_msg.orientation.w = q0;
			imu_msg.orientation.x = q1;
			imu_msg.orientation.y = q2;
			imu_msg.orientation.z = q3;

			set_timestamp(&imu_msg.header.stamp);

			RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
		}
	}
	#endif
	#ifdef MPU_6050
	if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
		Quaternion q;         // Кватернион
		VectorInt16 aa;       // Сырые данные акселерометра
		VectorInt16 aaReal;   // Ускорение без гравитации
		VectorInt16 gyro;     // Сырые данные гироскопа
		VectorFloat gravity;  // Вектор гравитации
	
		// Получение кватерниона
		mpu.dmpGetQuaternion(&q, fifoBuffer);
	
		// Получение сырых данных гироскопа (угловая скорость)
		mpu.dmpGetGyro(&gyro, fifoBuffer);
	
		// Получение сырых данных акселерометра и гравитации
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	
		// Заполнение сообщения IMU
		// Кватернион
		imu_msg.orientation.w = q.w;
		imu_msg.orientation.x = q.x;
		imu_msg.orientation.y = q.y;
		imu_msg.orientation.z = q.z;
	
		// Угловая скорость (в радианах/с)
		// MPU-6050 по умолчанию ±250°/с, чувствительность 131 LSB/(°/с)
		imu_msg.angular_velocity.x = (float)gyro.x / 131.0 * PI / 180.0;
		imu_msg.angular_velocity.y = (float)gyro.y / 131.0 * PI / 180.0;
		imu_msg.angular_velocity.z = (float)gyro.z / 131.0 * PI / 180.0;
	
		// Линейное ускорение (в м/с²)
		// MPU-6050 по умолчанию ±2g, чувствительность 16384 LSB/g, g ≈ 9.81 м/с²
		imu_msg.linear_acceleration.x = (float)aaReal.x / 16384.0 * 9.81;
		imu_msg.linear_acceleration.y = (float)aaReal.y / 16384.0 * 9.81;
		imu_msg.linear_acceleration.z = (float)aaReal.z / 16384.0 * 9.81;
	
		// Установка временной метки
		set_timestamp(&imu_msg.header.stamp);
	
		// Публикация
		RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
	}
	#endif
}

void lidar_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	if (lidar->dataObtained)
	{
		if (xSemaphoreTake(lidar->dataLock, portMAX_DELAY) == pdTRUE)
		{
			for(size_t i = 0; i < lidar_msg.ranges.capacity; i++)
				lidar_msg.ranges.data[i] = -1;

			for (int i = 0; i < lidar->dataSize; i++)
			{
				#ifndef LIDAR_INTENSITY
				if(lidar->intensity[i] < LIDAR_MIN_INTENSITY)
					continue;
				#endif

				uint16_t j = round(normalize_angle(lidar->theta[i]) * RAD_TO_ITER);
				
				if(j < lidar_msg.ranges.capacity) {
					lidar_msg.ranges.data[j] = lidar->distance[i] / 1000.0;
					#ifdef LIDAR_INTENSITY
					lidar_msg.intensities.data[j] = lidar->intensity[i];
					#endif
				}
			}

			lidar->dataObtained = false; // Сбрасываем флаг после вывода данных
			xSemaphoreGive(lidar->dataLock);

			set_timestamp(&lidar_msg.header.stamp);
			
			RCSOFTCHECK(rcl_publish(&lidar_publisher, &lidar_msg, NULL));
		}
	}
}

void logger_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
	while(MicroROSLogger::has_log_messages()) {
		LogMessage log = MicroROSLogger::get_next_log_message();
		
		log_msg.msg.data = log.message;
		log_msg.msg.size = strlen(log_msg.msg.data);

		log_msg.function.data = log.func;
		log_msg.function.size = strlen(log_msg.function.data);

		log_msg.file.data = log.file;
		log_msg.file.size = strlen(log_msg.file.data);

		log_msg.level = (uint8_t) log.level;

		set_timestamp(&log_msg.stamp);

		RCSOFTCHECK(rcl_publish(&log_publisher, &log_msg, NULL));
	}
}

void init_msgs_pid_params()
{
	pid_params_msg.data.capacity = 9;
	pid_params_msg.data.size = 9;
	pid_params_msg.data.data = new float[pid_params_msg.data.capacity];
}

void init_msgs_odometry()
{
	odom_msg.header.frame_id.capacity = 12;
	odom_msg.header.frame_id.size = 11;
	odom_msg.header.frame_id.data = new char[odom_msg.header.frame_id.capacity];
	odom_msg.header.frame_id = micro_ros_string_utilities_init("odom");

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
	imu_msg.header.frame_id = micro_ros_string_utilities_init("imu");
}

void init_msgs_lidar()
{
	lidar_msg.header.frame_id.capacity = 12;
	lidar_msg.header.frame_id.size = 11;
	lidar_msg.header.frame_id.data = new char[lidar_msg.header.frame_id.capacity];

	lidar_msg.header.frame_id = micro_ros_string_utilities_init("lidar");

	lidar_msg.ranges.capacity = RANGES_SIZE;
	lidar_msg.ranges.size = lidar_msg.ranges.capacity;
	lidar_msg.ranges.data = new float[lidar_msg.ranges.capacity];

	#ifdef LIDAR_INTENSITY
	lidar_msg.intensities.capacity = lidar_msg.ranges.capacity;
	lidar_msg.intensities.size = lidar_msg.intensities.capacity;
	lidar_msg.intensities.data = new float[lidar_msg.intensities.capacity];
	#endif

	lidar_msg.angle_min = 0;
	lidar_msg.angle_max = 2 * M_PI;
	lidar_msg.angle_increment = 2 * M_PI / lidar_msg.ranges.capacity;
	lidar_msg.range_min = 0.001;
	lidar_msg.range_max = 2.0;
}

void init_msgs_logger() {
	log_msg.msg.capacity = MESSAGE_LENGTH;
	log_msg.msg.size = 0;
	log_msg.msg.data = (char *)malloc(log_msg.msg.capacity * sizeof(char));

	log_msg.file.capacity = FILE_LENGTH;
	log_msg.file.size = 0;
	log_msg.file.data = (char *)malloc(log_msg.file.capacity * sizeof(char));
	
	log_msg.function.capacity = FUNC_LENGTH;
	log_msg.function.size = 0;
	log_msg.function.data = (char *)malloc(log_msg.msg.capacity * sizeof(char));
}

bool isDeviceConnected(uint8_t address) {
	Wire.beginTransmission(address); // Начинаем передачу на адрес устройства
	if (Wire.endTransmission() == 0) { // Если устройство ответило (0 - успех)
	  return true;
	}
	return false;
  }
  

void MicroRosController::microrosTask(void *pvParameters)
{
	IPAddress agent_ip_address = IPAddress();
	agent_ip_address.fromString(settings.agent_ip);

	static struct micro_ros_agent_locator locator;
	#ifdef AGENT_IP
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
	locator.address = getIPAddressByHostname(settings.agent_ip.c_str());
	locator.port = settings.agent_port;
	
	Serial.println(locator.address);
	Serial.println(locator.port);

	rmw_uros_set_custom_transport(
		false,
		(void *)&locator,
		platformio_transport_open,
		platformio_transport_close,
		platformio_transport_write,
		platformio_transport_read);
	#endif

	allocator = rcl_get_default_allocator();

	// Создаём настройки
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// Создаём ноду
	const char* node_name = "body_turtle_" + settings.turtle_id;
	RCCHECK(rclc_node_init_default(&node, node_name, "", &support));

    // Синхронизация времени с агентом
    while(rmw_uros_sync_session(1000) != RMW_RET_OK)
		vTaskDelay(100);

	// Создаём подписчиков, паблишеры и таймеры
	rmw_qos_profile_t cmd_qos = rmw_qos_profile_sensor_data;

	RCCHECK(rclc_subscription_init(
		&cmd_vel_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"/cmd_vel",
		&cmd_qos
	));

	rmw_qos_profile_t pid_params_qos = rmw_qos_profile_default;
	pid_params_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
	pid_params_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

	RCCHECK(rclc_subscription_init(
		&pid_params_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
		"/params",
		&pid_params_qos
	));

	rmw_qos_profile_t odometry_qos = rmw_qos_profile_default;

	RCCHECK(rclc_publisher_init(
		&odom_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
		"/odom",
		&odometry_qos
	));

	RCCHECK(rclc_timer_init_default(
		&odom_timer,
		&support,
		RCL_MS_TO_NS(settings.odom_delay),
		odometry_timer_callback));

	rmw_qos_profile_t imu_qos = rmw_qos_profile_default;

	RCCHECK(rclc_publisher_init(
		&imu_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"/imu",
		&imu_qos
	));

	RCCHECK(rclc_timer_init_default(
		&imu_timer,
		&support,
		RCL_MS_TO_NS(settings.imu_delay),
		imu_timer_callback));

	rmw_qos_profile_t lidar_qos = rmw_qos_profile_default;

	RCCHECK(rclc_publisher_init(
		&lidar_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
		"/lidar",
		&lidar_qos
	));

	RCCHECK(rclc_timer_init_default(
		&lidar_timer,
		&support,
		RCL_MS_TO_NS(settings.lidar_delay),
		lidar_timer_callback));

	rmw_qos_profile_t rosout_qos = rmw_qos_profile_services_default;
	rosout_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
	rosout_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

	RCCHECK(rclc_publisher_init(
		&log_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
		"/rosout", &rosout_qos));

	RCCHECK(rclc_timer_init_default(
		&log_timer,
		&support,
		RCL_MS_TO_NS(settings.logger_delay),
		logger_timer_callback));

	init_msgs_lidar();
	init_msgs_imu();
	init_msgs_odometry();
	init_msgs_pid_params();
	init_msgs_logger();

	RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &log_timer));


	lidar = new DreameLidar(&LIDAR_SERIAL);
	xTaskCreate(DreameLidar::lidarTask, "lidar_data_task", 8196, lidar, 1, NULL);
	RCCHECK(rclc_executor_add_timer(&executor, &lidar_timer));

	INIT_ENCODER_WITH_NAME(LEFT, ENCODER_M1_A, ENCODER_M1_B)
	INIT_ENCODER_WITH_NAME(RIGHT, ENCODER_M2_A, ENCODER_M2_B)

	motorA.setPIDConfig(2.0, 0.0, 0.06, 2.25);
	motorB.setPIDConfig(2.0, 0.0, 0.06, 2.25);

	RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &pid_params_subscriber, &pid_params_msg, &pid_params_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));

	#ifdef MPU_9250
	if (Wire.begin() && isDeviceConnected(0x68)) {
		if (imu.begin() != INV_SUCCESS)
		{
			Serial.println("Unable to communicate with MPU-9250");
			Serial.println("Check connections, and try again.");
		} else {
			imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
			imu.setAccelFSR(2);
			imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT |
							DMP_FEATURE_GYRO_CAL |
							DMP_FEATURE_SEND_CAL_GYRO |
							DMP_FEATURE_SEND_RAW_ACCEL,
							10);
			imu.dmpSetOrientation(orientationDefault);

			RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
		}
	} else {
		Serial.println("Unable to communicate with I2C");
	}
	#endif
	#ifdef MPU_6050
	if(Wire.begin()) {
		//Wire.setClock(1000000UL);   // разгоняем шину на максимум
		// инициализация DMP
		mpu.initialize();
		mpu.dmpInitialize();
		mpu.setDMPEnabled(true);

		RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
	}
	#endif

	Serial.println("ROS started!");
	while (true)
	{
		RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
		vTaskDelay(1);
	}
}

