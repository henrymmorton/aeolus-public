#ifndef tcontrol_h
#define tcontrol_h

#include "Arduino.h"
#include <cmath>

const float Pd = 5;
const float Pi = 1;

struct TeensyControl {
    float speed_command = 0;
    float steering_command = 0;
};

const size_t CONTROL_SIZE = sizeof(TeensyControl);
const uint8_t HEADER[2] = {0xAA, 0x55};

/**
 * Read the control signal from the provided serial
 * @param serial The serial port that contains the control signal
 * @param raw_control A reference to a fresh teensy control object
*/
bool readControlSignalFromSerial(HardwareSerial &serial, TeensyControl &raw_control);

/**
 * @param my_control The control signal to print
 * @param serial The serial print to print the signal to
 */
void printControlSignal(const TeensyControl &my_control, Stream &serial);

class PIController {
    public:
        static constexpr float throttle_neutral = 1535;
        static constexpr float throttle_max = 2000;
        static constexpr float throttle_min = 1000;

        PIController(double kp, double ki)
            : kp_(kp), ki_(ki), integral_(0.0), t_prev_(0.0) {}

        double update(float speed_command, float measured_speed);

        void reset();

    private:
        double kp_;
        double ki_;
        double t_prev_;
        double integral_;
};

class SteeringController {
    public:
        static constexpr float rad_2_deg = 180 / PI;
        static constexpr float steering_neutral = 90;
        static constexpr float steering_gain = 5;

        float steering_angle_to_servo(float steering_angle);
};

#endif

