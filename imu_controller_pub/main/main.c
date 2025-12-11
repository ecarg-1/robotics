#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "mpu9250.h"
#include "driver/i2c.h"
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/vector3.h>
#include <math.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t pitch_roll_publisher;
geometry_msgs__msg__Vector3 vector3_msg;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		int16_t ax, ay, az, gx, gy, gz, temp;
        mpu9250_read_imu_raw(&ax, &ay, &az, &gx, &gy, &gz, &temp);

		//scale aclleration to m/s^2
		float ax_g = ax * (2.0f / 32768.0f);
		float ay_g = ay * (2.0f / 32768.0f);
		float az_g = az * (2.0f / 32768.0f);

		imu_msg.linear_acceleration.x = ax_g * 9.81f;
		imu_msg.linear_acceleration.y = ay_g * 9.81f;
		imu_msg.linear_acceleration.z = az_g * 9.81f;

		//scale gyro to rad/s
		float gx_r = gx * ((250.0f / 32768.0f) * M_PI/180.0f);
		float gy_r = gy * ((250.0f / 32768.0f) * M_PI/180.0f);
		float gz_r = gz * ((250.0f / 32768.0f) * M_PI/180.0f);

		imu_msg.angular_velocity.x = gx_r;
		imu_msg.angular_velocity.y = gy_r;
		imu_msg.angular_velocity.z = gz_r;

		//compute pitch + roll
		float pitch = atan2f(ax_g, sqrtf(ay_g * ay_g + az_g * az_g));
		float roll  = atan2f(ay_g, sqrtf(ax_g * ax_g + az_g * az_g));

		vector3_msg.z = 0;
		vector3_msg.x = pitch;
		vector3_msg.y = roll;

		RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
		RCSOFTCHECK(rcl_publish(&pitch_roll_publisher, &vector3_msg, NULL));
	}
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create pub_node
	rcl_node_t pub_node;
	RCCHECK(rclc_node_init_default(&pub_node, "pub_node", "", &support));

	// create imu_publisher
	RCCHECK(rclc_publisher_init_default(
		&imu_publisher,
		&pub_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"/imu"));

	//pitch roll publisher
	RCCHECK(rclc_publisher_init_default(
		&pitch_roll_publisher,
		&pub_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
		"/pitch_roll"
	));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 20;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	
	sensor_msgs__msg__Imu__init(&imu_msg); //initialize message because there are so many fields
	vector3_msg.x = vector3_msg.y = vector3_msg.z = 0.0;

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&imu_publisher, &pub_node));
	RCCHECK(rcl_publisher_fini(&pitch_roll_publisher, &pub_node));
	RCCHECK(rcl_node_fini(&pub_node));

  	vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif
	// init imu
	mpu9250_i2c_init();
	mpu9250_init();
	
    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}
