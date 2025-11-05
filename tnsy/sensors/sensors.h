#ifndef sensors_h
#define sensors_h

#include "Arduino.h"
#include "ICM_20948.h" //Inlcude accelerometer library
#include "HCSR04.h" //include the ultrasonic sensor library
#include "teensy_pins.h"

#define IMUI2C 1      // The value of the last bit of the I2C address. On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0

class IMUData {
    private:

    public:
    float x_acc;
    float y_acc;
    float z_acc;
    float yaw_rate;
    float pitch_rate;
    float roll_rate;
    double yaw;
    double pitch;
    double roll;

    bool new_accel_data;
    bool new_euler_data;

    IMUData() {
        x_acc = 0;
        y_acc = 0;
        z_acc = 0;
        yaw_rate = 0;
        pitch_rate = 0;
        roll_rate = 0;
        yaw = 0;
        pitch = 0;
        roll = 0;

        new_accel_data = false;
        new_euler_data = false;
    }

    void serialPrintIMUData();
};

/*IMU sensor class*/
class IMU {
    private:
    ICM_20948_I2C imu;
    const float mg2mps = 0.00981; // The constant ratio between milli-gs and meters per second

    public:
    IMUData current_imu_data;

    IMU() : current_imu_data() {
    }

    void initIMU();

    IMUData& getIMUData();

    void updateIMUData();

    void serialPrintIMUData();

};

class Tachometer {
    public:
    int hall_pin;
    int rev_thrsh = 25;

    bool hall_trpd;
    float rev_count;
    bool new_dat;
    
    double t1;
    double dt_micros;
    double dt_s;

    float rpm;

    Tachometer(int tach_pin) {
        hall_pin = tach_pin;
        rev_count = 0;
        new_dat = false;
        rpm = 0;
    }

    void initTachometer();

    void simpleRead();

    void updateTachometer();

    void printRPM();
    
};

class UltrasonicData {
    public:
    
    double ob_dist;  // The distance detected by the ultrasonic sensor
    bool detected; //

    UltrasonicData() {
        ob_dist = 400;
        detected = false;
    }

    void processRawDistance(float raw_distance);

    void printUltrasonicData();

};

#endif



