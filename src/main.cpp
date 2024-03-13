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

// #include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/float32.h>

#define SSID "C3_805"
#define PASSWORD "nikolakis1995"

// Define BUILTIN LED PIN
#define LED_PIN LED_BUILTIN
// Define the control inputs
#define MOT_AIN1_FWD 1
#define MOT_AIN2_REV 2
#define MOT_BIN1_FWD 3
#define MOT_BIN2_REV 4

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// subscriber
// std_msgs__msg__UInt32 msg;
std_msgs__msg__Float32 msg;
// rcl_subscription_t subscriber;
rcl_subscription_t subscriber_1;
rcl_subscription_t subscriber_2;
rclc_executor_t executor_sub;

// void set_motor_pwm(uint8_t pwm_fwd_1, uint8_t pwm_rev_1, uint8_t pwm_fwd_2, uint8_t pwm_rev_2);

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
 * @brief loop to indicate error with blinking LED
 *
 */
void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

/**
 * @brief subscription callback executed when receiving a message
 *
 * @param msgin
 */
void subscriber_callback_1(const void *msgin)
{
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float wheel_vel = msg->data;
  if (wheel_vel > 0)
  {
    analogWrite(MOT_AIN1_FWD, (int) 255 * wheel_vel/25.08361);
    analogWrite(MOT_AIN2_REV, 0);
  }
  else
  {
    analogWrite(MOT_AIN1_FWD, 0);
    analogWrite(MOT_AIN2_REV, (int) 255 * wheel_vel/25.08361);
  }
}

/**
 * @brief subscription callback executed when receiving a message
 *
 * @param msgin
 */
void subscriber_callback_2(const void *msgin)
{
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float wheel_vel = msg->data;
  if (wheel_vel > 0)
  {
    analogWrite(MOT_BIN1_FWD, (int) 255 * wheel_vel/25.08361);
    analogWrite(MOT_BIN2_REV, 0);
  }
  else
  {
    analogWrite(MOT_BIN1_FWD, 0);
    analogWrite(MOT_BIN2_REV, (int) 255 * wheel_vel/25.08361);
  }
}

/**
 * @brief subscription callback executed at receiving a message
 *
 * @param msgin
 */
// void subscriber_callback(const void *msgin)
// {
//   const std_msgs__msg__UInt32 *msg = (const std_msgs__msg__UInt32 *)msgin;
//   uint8_t pwm_fwd_1,pwm_rev_1,pwm_fwd_2,pwm_rev_2;
//   pwm_fwd_1 = msg->data >> 24;
//   pwm_rev_1 = (msg->data & 0x00FF0000) >> 16;
//   pwm_fwd_2 = (msg->data & 0x0000FF00) >> 8;
//   pwm_rev_2 = (msg->data & 0x000000FF);

//   // (condition) ? (true exec):(false exec)
//   digitalWrite(LED_PIN, ( pwm_fwd_2 == 118) ? LOW : HIGH);
//   set_motor_pwm(pwm_fwd_1,pwm_rev_1,pwm_fwd_2,pwm_rev_2);
// }

// void set_motor_pwm(uint8_t pwm_fwd_1, uint8_t pwm_rev_1, uint8_t pwm_fwd_2, uint8_t pwm_rev_2){
//   // Set motor directions
//   analogWrite(MOT_AIN1_FWD, pwm_fwd_1);
//   analogWrite(MOT_AIN2_REV, pwm_rev_1);
//   analogWrite(MOT_BIN1_FWD, pwm_fwd_2);
//   analogWrite(MOT_BIN2_REV, pwm_rev_2);
// }

void setup()
{
  // Initialize the LED_PIN
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

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

  // Allow some time for everything to start properly
  delay(2000);

  // Get the default memory allocator provided by rcl
  allocator = rcl_get_default_allocator();

  // Initialize rclc_support with default allocator
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Initialize a ROS node with the name "micro_ros_platformio_node"
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // Initialize a ROS subscriber with the name "micro_ros_platformio_subscriber_node" to Subscribe(Receive) UInt32 messages
  // RCCHECK(rclc_subscription_init_default
  // (
  //   &subscriber,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
  //   "wheel_velocity")
  // );

  // Create 1st subscriber
  RCCHECK(rclc_subscription_init_default
  (
    &subscriber_1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/left_wheel_vel")
  );

  // Create 2nd subscriber
  RCCHECK(rclc_subscription_init_default
  (
    &subscriber_2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/right_wheel_vel")
  );

  // create executor
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber_1, &msg, &subscriber_callback_1, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber_2, &msg, &subscriber_callback_2, ON_NEW_DATA));


  // Initialize an executor that will manage the execution of all the ROS entities (publishers, subscribers, services, timers)
  // RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  // Add subscription executor (execute callback when new data received)
  // RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg, &subscriber_callback, ON_NEW_DATA));
}

void loop()
{
  // delay(10);
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
}