// this code is example of publisher and subscriber
// https://github.com/micro-ROS/micro_ros_arduino/blob/galactic/examples/micro-ros_subscriber/micro-ros_subscriber.ino
// https://github.com/micro-ROS/micro_ros_arduino/blob/galactic/examples/micro-ros_publisher/micro-ros_publisher.ino
// run micro_ros_agent before connecting arduino
// docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:galactic serial --dev /dev/cu.usbmodem1422201 -v6

#include <Arduino.h>
// The micro_ros_platformio library provides the functions to communicate with ROS2
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// Define the control inputs
#define MOT_AIN1_FWD 1
#define MOT_AIN2_REV 2
#define MOT_BIN1_FWD 3
#define MOT_BIN2_REV 4

#define LOWER_PWM_VALUE 80
#define VEHICLE_MAX_SPEED 25.08361

// // Define robot length and wheel radius  
// #define R 0.0598
// #define L 0.109

#define SSID "C3_805"
#define PASSWORD "nikolakis1995"

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// Message
geometry_msgs__msg__Twist msg;

// Subscriber
rcl_subscription_t cmd_vel_subscriber;
rclc_executor_t executor_sub;


#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

/**
 * @brief loop to indicate error
 *
 */
// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

/**
 * @brief Convert the commanded velocity to PWM.
 *
 * @param[in] x the commanded velocity.
 * 
 * @return the PWM value. 
 */
int velocity_2_pwm(float x)
{
  int y;
  int a;
  int b = LOWER_PWM_VALUE;
  a = (225 - LOWER_PWM_VALUE) / VEHICLE_MAX_SPEED;
  y = a*x + b;
  return y;
}

/**
 * @brief subscription callback executed when receiving a message
 *
 * @param msgin
 */
void cmd_vel_callback(const void *msgin)
{
  float R = 0.0598;
  float L = 0.109;
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  float ux = msg->linear.x;
  float w = - msg->angular.z;

  // Left wheel velocity
  // υL = (2 * ux + ω*L)/(2*R)
  // Right wheel velocity
  // υR = (2 * ux - ω*L)/(2*R)
  // ux is linear.x
  // ω is angular.z
  // L is the distance between the wheels
  // R is the radius of the wheel
  
  // Left wheel velocity
  float uL = ((2 * ux) + (w * L)) / (2 * R);
  // Right wheel velocity
  float uR = ((2 * ux) - (w * L)) / (2 * R);
  int pwm_left;
  int pwm_right;

  pwm_left = velocity_2_pwm(abs(uL));
  pwm_right = velocity_2_pwm(abs(uR));
  
  if (uL > 0)
  {
    analogWrite(MOT_AIN1_FWD, pwm_left);
    analogWrite(MOT_AIN2_REV, 0);
  }
  else if (uL == 0){
    analogWrite(MOT_AIN1_FWD, 0);
    analogWrite(MOT_AIN2_REV, 0);
  }
  else
  {
    analogWrite(MOT_AIN1_FWD, 0);
    analogWrite(MOT_AIN2_REV, pwm_left);
  }

  if (uR > 0)
  {
    analogWrite(MOT_BIN1_FWD, pwm_right);
    analogWrite(MOT_BIN2_REV, 0);
  }
  else if (uR == 0)
  {
    analogWrite(MOT_BIN1_FWD, 0);
    analogWrite(MOT_BIN2_REV, 0);
  }
  else
  {
    analogWrite(MOT_BIN1_FWD, 0);
    analogWrite(MOT_BIN2_REV, pwm_right);
  }
}

void setup()
{

  // Set all the motor control inputs to OUTPUT
  pinMode(MOT_AIN1_FWD, OUTPUT);
  pinMode(MOT_AIN2_REV, OUTPUT);
  pinMode(MOT_BIN1_FWD, OUTPUT);
  pinMode(MOT_BIN2_REV, OUTPUT);

  // Turn off motors - Initial state
  analogWrite(MOT_AIN1_FWD, 0);
  analogWrite(MOT_AIN2_REV, 0);
  analogWrite(MOT_BIN1_FWD, 0);
  analogWrite(MOT_BIN2_REV, 0);

  // Declare Agent's IP
  IPAddress agent_ip(192, 168, 70, 178);
  // Declare Agent's Port
  size_t agent_port = 8888;

  // Enter SSID and Password
  char ssid[] = SSID;
  char psk[]= PASSWORD;

  // Configure Micro-ROS library to use Wi-Fi for communication
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

  // Configure serial transport
  // Serial.begin(115200);
  // set_microros_serial_transports(Serial);
  delay(2000);


  // Get the default memory allocator provided by rcl
  allocator = rcl_get_default_allocator();

  // Initialize rclc_support with default allocator
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Initialize a ROS node with the name "micro_ros_platformio_node"
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // Create Subscriber
  RCCHECK(rclc_subscription_init_default
  (
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel")
  );

  // Initialize an executor that will manage the execution of all the ROS entities (publishers, subscribers, services, timers)
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  // Add subscription executor (execute callback when new data received)
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &cmd_vel_subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA));
}

void loop() {
  rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(1));
}
