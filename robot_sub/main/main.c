//references
//https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html#_CPPv411gpio_configPK13gpio_config_t
//https://github.com/espressif/esp-idf/blob/master/examples/peripherals/ledc/ledc_basic/main/ledc_basic_example_main.c
//https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html#api-reference
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/vector3.h>
#include "driver/ledc.h"
#include <math.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define IN1 					GPIO_NUM_32
#define IN2 					GPIO_NUM_33
#define IN3 					GPIO_NUM_25
#define IN4 					GPIO_NUM_26
#define PWM_L 					GPIO_NUM_27
#define PWM_R 					GPIO_NUM_14

#define LEDC_TIMER 				LEDC_TIMER_0
#define LEDC_MODE 				LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_L          LEDC_CHANNEL_0
#define LEDC_CHANNEL_R          LEDC_CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_MAX_DUTY   		8192 //speed cap
#define LEDC_MIN_DUTY   		5500 //min to have motors spin
#define LEDC_CLK_SRC            LEDC_AUTO_CLK
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz


rcl_subscription_t robot_sub;
rcl_publisher_t robot_pub;
geometry_msgs__msg__Vector3 vector3_sub;
geometry_msgs__msg__Vector3 vector3_pub;
float global_lf = 0.0f; 
float global_rf = 0.0f;
float global_lpwm = 0.0f;
float global_rpwm = 0.0f;


void init_ledc(void){
	//left motor
	ledc_timer_config_t timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY, 
        .clk_cfg          = LEDC_CLK_SRC,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t channel_l = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_L,
        .timer_sel      = LEDC_TIMER,
        .gpio_num       = PWM_L,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_l));

	//right motor
    ledc_channel_config_t channel_r = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_R,
        .timer_sel      = LEDC_TIMER,
        .gpio_num       = PWM_R,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_r));
}

void set_motors(int motor_num, float factor){
	//left=0(in1,in2), right=1(in3,in4)
	//in1=1, in2=0 = forward, in1=0, in2=1 = backward

	//clamping
	if (factor >= 1.0f) factor = 1.0f;
	if (factor <= -1.0f) factor = -1.0f;
	
	//setting direction and speed 
	int32_t duty = LEDC_MIN_DUTY + fabsf(factor)*(LEDC_MAX_DUTY-LEDC_MIN_DUTY);
	if (fabsf(factor) < .2f) duty = 0.0f; //dead band
	if (duty != 0.0f){
		if (motor_num == 1){ //right
		gpio_set_level(IN3, factor >= 0); 
		gpio_set_level(IN4, factor < 0);
		ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_R, duty);
		ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_R);
		global_rpwm = duty;
		}
	if (motor_num == 0){ //left
		gpio_set_level(IN1, factor >= 0); 
		gpio_set_level(IN2, factor < 0);
		ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_L, duty);
		ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_L);
		global_lpwm = duty;
		}
	}
	else { //stop motors
		gpio_set_level(IN1, 0); 
		gpio_set_level(IN2, 0);
		gpio_set_level(IN3, 0); 
		gpio_set_level(IN4, 0);
		global_lpwm = global_rpwm = duty;

	}
	
}

void robot_callback(const void * msgin){
	const geometry_msgs__msg__Vector3 * vector3_sub = (const geometry_msgs__msg__Vector3 *)msgin;
	float pitch = vector3_sub->x;
	float roll = vector3_sub->y;

	//clamping
	if (pitch >= 1.0f) pitch = 1.0f;
	if (pitch <= -1.0f) pitch = -1.0f;
	if (roll >= 1.0f) roll = 1.0f;
	if (roll <= -1.0f) roll = -1.0f;

	//dead band
	if (fabsf(pitch) < .15) pitch = 0.0f;
	if (fabsf(roll) < .2) roll = 0.0f;

	//factors for the math
	float gain_p = 1.0f;
	float gain_r = 0.8f; 

	float wheel_r = pitch*gain_p + roll*gain_r;
	float wheel_l = pitch*gain_p - roll*gain_r;

	global_lf = wheel_l;
	global_rf = wheel_r;

	set_motors(0, wheel_l);
	set_motors(1, wheel_r);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		
		vector3_pub.x =  global_lf;
		vector3_pub.y =  global_lpwm;
		vector3_pub.z  =  global_rf;
		//  =  global_rpwm;
		RCSOFTCHECK(rcl_publish(&robot_pub, &vector3_pub, NULL));
	}
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// Create init_options.
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif
	// Setup support structure.
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// Create node.
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "robot_node", "", &support));

	// Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&robot_sub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
		"/pitch_roll"));

	//create publisher
	RCCHECK(rclc_publisher_init_default(
		&robot_pub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
		"/wheel_speeds"));

	// Create executor.
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
	
	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 20;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// Add timer and subscriber to executor.
	geometry_msgs__msg__Vector3__init(&vector3_sub);
	geometry_msgs__msg__Vector3__init(&vector3_pub);
	RCCHECK(rclc_executor_add_subscription(&executor, &robot_sub, &vector3_sub, &robot_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	

	// Spin forever.
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
	}

	// Free resources.
	RCCHECK(rcl_subscription_fini(&robot_sub, &node));
	RCCHECK(rcl_publisher_fini(&robot_pub, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif
	//setting up pins for h bridge and pwm
	gpio_reset_pin(IN1); 
	gpio_reset_pin(IN2); 
	gpio_reset_pin(IN3); 
	gpio_reset_pin(IN4); 
	gpio_set_direction(IN1, GPIO_MODE_OUTPUT);
	gpio_set_direction(IN2, GPIO_MODE_OUTPUT);
	gpio_set_direction(IN3, GPIO_MODE_OUTPUT);
	gpio_set_direction(IN4, GPIO_MODE_OUTPUT);

	init_ledc();
	ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_L, 0);
	ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_R, 0);
	ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_L);
	ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_R);

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}