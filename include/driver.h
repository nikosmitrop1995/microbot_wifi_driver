#include <Arduino.h>
#include <geometry_msgs/msg/twist.h>

// Define the control inputs
#define MOT_AIN1_FWD A1
#define MOT_AIN2_REV A2
#define MOT_BIN1_FWD A3
#define MOT_BIN2_REV A4
#define SLP D5


class Driver
{
  public:
    Driver();

    void setup();

    /**
     * @brief subscription callback executed when receiving a message
     *
     * @param msgin
     */
    static void cmd_vel_callback(const void *msgin);

    /**
     * @brief Convert the commanded velocity to PWM.
     *
     * @param[in] x the commanded velocity.
     * 
     * @return the PWM value. 
     */
    static int velocity_2_pwm(float x);

  protected:

  private:

};