#include "driver.h"

Driver::Driver(){};

void Driver::setup(){
  
  // Set all the motor control inputs to OUTPUT
  pinMode(MOT_AIN1_FWD, OUTPUT);
  pinMode(MOT_AIN2_REV, OUTPUT);
  pinMode(MOT_BIN1_FWD, OUTPUT);
  pinMode(MOT_BIN2_REV, OUTPUT);
  pinMode(SLP, OUTPUT);

  // Turn off motors - Initial state
  analogWrite(MOT_AIN1_FWD, 0);
  analogWrite(MOT_AIN2_REV, 0);
  analogWrite(MOT_BIN1_FWD, 0);
  analogWrite(MOT_BIN2_REV, 0);
  digitalWrite(SLP, HIGH);
}


int Driver::velocity_2_pwm(float x)
{
  int a;
  int lower_pwm_value = 80; // This has to be converted to param 
  int y;
  float vehicle_max_speed = 25.08361; // This has to be converted to param 

  a = (225 - lower_pwm_value) / vehicle_max_speed;
  y = a*x + lower_pwm_value;
  return y;
}

void Driver::cmd_vel_callback(const void *msgin)
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

  pwm_left = Driver::velocity_2_pwm(abs(uL));
  pwm_right = Driver::velocity_2_pwm(abs(uR));
  
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
