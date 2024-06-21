#include "driver.h"
#include "uros.h"


// Declare uros pointer
URos* uros = nullptr;

void setup()
{
  Driver driver;
  driver.setup();

  // Allocate memory for uros object
  uros = new URos;
  char ssid[] = "C3_805";
  char psk[] = "nikolakis1995";
  std::stringstream ip_address("192.168.70.178");
  uint16_t port = 8888;
  uros->connect_to_wifi(ssid, psk, ip_address, port);

  uros->initialize();
  // Add subscription executor (execute callback when new data received)
  RCCHECK(rclc_executor_add_subscription(&(uros->executor_sub), &(uros->cmd_vel_subscriber), &(uros->msg), &driver.cmd_vel_callback, ON_NEW_DATA));
}

void loop() {
  // Check if uros is initialized
  if (uros) {
    rclc_executor_spin_some(&(uros->executor_sub), RCL_MS_TO_NS(1));
  }
}
