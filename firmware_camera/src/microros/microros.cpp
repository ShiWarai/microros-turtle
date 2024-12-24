#include "microros/microros.hpp"

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rcl_timer_t timer;
std_msgs__msg__Int32 msg;

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
		delay(100);
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
	RCCHECK(rclc_node_init_default(&node, String("cam_node_tutle_" + settings.turtle_id).c_str(), "", &support));


	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"test_pub"));
	
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"test_sub"));

	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); // create executor
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

	msg.data = 0;
	
    while(true) {
        delay(100);
	    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    }
}