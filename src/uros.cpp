#include "uros.h"

URos::URos(){
}

URos::~URos(){}

std::vector<int> URos::split_ip(std::stringstream &ip_address){

  std::string segment;
  std::vector<int> ip_vector;
  while(std::getline(ip_address, segment, '.'))
  {
    ip_vector.push_back(std::stoi(segment));
  }
  return ip_vector;
}

void URos::connect_to_wifi(char * ssid, char * psk, std::stringstream &ip_address, uint16_t &port){

  std::vector<int> ip_vector = URos::split_ip(ip_address);
  IPAddress agent_ip(ip_vector[0], ip_vector[1], ip_vector[2], ip_vector[3]);

  // IPAddress agent_ip(192, 168, 70, 178);
  // Configure Micro-ROS library to use Wi-Fi for communication
  set_microros_wifi_transports(ssid, psk, agent_ip, port);
  delay(2000);

}

void URos::initialize(){

  // Get the default memory allocator provided by rcl
  allocator = rcl_get_default_allocator();

  // Initialize rclc_support with default allocator
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Initialize a ROS node with the name "micro_ros_platformio_node"
  RCCHECK(rclc_node_init_default(&node, "microbot_controller_node", "", &support));

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
}
