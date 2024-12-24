#include "microros/microros.hpp"

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rcl_timer_t timer;
std_msgs__msg__Int32 msg;

rcl_subscription_t move_vector_subscriber;
geometry_msgs__msg__Twist move_vector_msg;

DreameLidar *lidar;
rcl_publisher_t lidar_publisher;
rcl_timer_t lidar_timer;
sensor_msgs__msg__LaserScan lidar_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

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

void error_loop() {
	while(1) {
		Serial.println("Error!");
		vTaskDelay(100);
	}
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		msg.data++;
	}
}

void subscription_callback(const void *msgin)
{  
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	Serial.println(msg->data);
}

void move_vector_callback(const void *msgin)
{  
	const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
	Serial.println(msg->linear.x);
}

void lidar_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
        if (lidar->dataObtained) {
			// for(size_t i = 0; i < lidar_msg.ranges.capacity; i++)
			// 	lidar_msg.ranges.data[i] = 0;
            if (xSemaphoreTake(lidar->lock, portMAX_DELAY) == pdTRUE) {
				uint8_t deg;
				for(int i = 0; i < lidar->dataSize; i++) {
					deg = round(-lidar->theta[i] * 180.0 / M_PI);
					if(lidar->theta[i] < lidar_msg.angle_max && lidar->theta[i] > lidar_msg.angle_min) {
						//Serial.printf("%d: %f %d;\r\n", deg, lidar->theta[i], lidar->distance[i]);
						lidar_msg.header.stamp.nanosec = esp_timer_get_time() * 1000;
						lidar_msg.ranges.data[deg] = ((float)lidar->distance[i]) / 1000.0;
						#ifdef LIDAR_INTENSITY
						lidar_msg.intensities.data[deg] = lidar->intensity[i];
						#endif
					}
				}

                lidar->dataObtained = false; // Сбрасываем флаг после вывода данных
                xSemaphoreGive(lidar->lock);
            }
        }

		RCSOFTCHECK(rcl_publish(&lidar_publisher, &lidar_msg, NULL));
	}
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

    lidar_msg.angle_min = -2*M_PI;
    lidar_msg.angle_max = 0;
    lidar_msg.angle_increment = 2*M_PI / lidar_msg.ranges.capacity;
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
			(void *) &locator,
			platformio_transport_open,
			platformio_transport_close,
			platformio_transport_write,
			platformio_transport_read
		);
	#else
		#error "NEED FIX THIS BLOCK"
		set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, getIPAddressByHostname(AGENT_HOSTNAME), AGENT_PORT);
	#endif

	allocator = rcl_get_default_allocator();

	//create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	RCCHECK(rclc_node_init_default(&node, String("body_node_tutle_" + settings.turtle_id).c_str(), "", &support));


	// RCCHECK(rclc_publisher_init_default(
	// 	&publisher,
	// 	&node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
	// 	"micro_ros_publisher"));
	
	// const unsigned int timer_timeout = 1000;
	// RCCHECK(rclc_timer_init_default(
	// 	&timer,
	// 	&support,
	// 	RCL_MS_TO_NS(timer_timeout),
	// 	timer_callback));

	// RCCHECK(rclc_subscription_init_default(
	// 	&subscriber,
	// 	&node,
	// 	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
	// 	"micro_ros_subscriber"));

	RCCHECK(rclc_subscription_init_best_effort(
		&move_vector_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"move_vector"));

	RCCHECK(rclc_publisher_init_default(
		&lidar_publisher,
		&node,	
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
		"lidar"));

	const unsigned int lidar_timer_timeout = 200;
	RCCHECK(rclc_timer_init_default(
		&lidar_timer,
		&support,
		RCL_MS_TO_NS(lidar_timer_timeout),
		lidar_timer_callback));

	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); // create executor
	// RCCHECK(rclc_executor_add_timer(&executor, &timer));
	// RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &move_vector_subscriber, &move_vector_msg, &move_vector_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &lidar_timer));

	msg.data = 0;

	lidar = new DreameLidar(&Serial2);
	init_msgs_laserscan();
    lidar->startTask();
	
    while(true) {
        delay(100);
	    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    }
}