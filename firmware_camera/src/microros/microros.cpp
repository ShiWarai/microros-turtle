#include "microros/microros.hpp"

rcl_publisher_t image_publisher;
rcl_timer_t image_timer;
sensor_micro_msgs__msg__CompressedImage image_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_VGA,
    .jpeg_quality = 10,
    .fb_count = 3,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY
};

void send_image() 
{
    camera_fb_t *image = esp_camera_fb_get();

    if (image) {
        if (image->len > image_msg.data.capacity) {
            if (image_msg.data.data != nullptr) {
                free(image_msg.data.data);
            }

            image_msg.data.data = (uint8_t *)malloc(image->len * sizeof(uint8_t));
            image_msg.data.capacity = image->len;
        }

        image_msg.data.size = image->len;
        memcpy(image_msg.data.data, image->buf, image->len);

        RCSOFTCHECK(rcl_publish(&image_publisher, &image_msg, NULL));

        esp_camera_fb_return(image);
    } else {
        Serial.println("Fail to capture image");
    }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    send_image();
}

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
        &image_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_micro_msgs, msg, CompressedImage),
        "/camera/image_raw"
    ));

    const unsigned int timer_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &image_timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));
	

	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); // create executor
    #if CAMERA_MODE == 1
    RCCHECK(rclc_executor_add_timer(&executor, &image_timer));
    #endif

    #if CAMERA_MODE == 1
    // Initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.println("Camera init failed");
    }

    // Allocate initial memory for the img_msg (will be resized later if needed)
    image_msg.data.data = nullptr;
    image_msg.data.capacity = 0;
    image_msg.data.size = 0;

    // Assign frame_id and format
    image_msg.header.frame_id.data = (char *)malloc(10 * sizeof(char));
    strcpy(image_msg.header.frame_id.data, "camera_frame");
    image_msg.header.frame_id.size = strlen("camera_frame");
    image_msg.header.frame_id.capacity = 10;

    image_msg.format.data = (char *)malloc(5 * sizeof(char));
    strcpy(image_msg.format.data, "jpeg");
    image_msg.format.size = strlen("jpeg");
    image_msg.format.capacity = 5;
    #endif

    Serial.println("Initialization microROS complete!");
	
    while(true) {
	    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
        vTaskDelay(1);
    }
}