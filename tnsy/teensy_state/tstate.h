#ifndef tstate_h
#define tstate_h

#include "Arduino.h"
#include "math.h"
#include "sensors.h"

class TeensyState {
    private:
    // TODO: Measure these
    const float gear_ratio = 4;
    const float wheel_radius = 0.1;

    public:
    double time;

    /* IMU variables */
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

    /* Ultrasonic Variables*/
    double obstc_dist;
    bool obstc_detected;

    /*Tachometer Variables*/
    double rpm;
    bool new_tachdat;

    /*Calulated Variables*/
    float velocity_mag;
    float vel_gx; //The velocity in the global x direction
    float vel_gy; //The velocity in the global y direction
    float acc_mag;

    TeensyState() {
        time = micros();

        x_acc = 0;
        y_acc = 0;
        z_acc = 0;
        yaw_rate = 0;
        pitch_rate = 0;
        roll_rate = 0;
        yaw = 0;
        pitch = 0;
        roll = 0;

        obstc_dist = 0;
        obstc_detected = false;

        rpm = 0;

        velocity_mag = 0;
        vel_gx = 0;
        vel_gy = 0;
        acc_mag = 0;
    }

    /**
     * Calculates the velocity magnitude and global components from the rpm and yaw.
     */
    void calcVelocity();

    /** 
     * Calculates the acceleration magnitude from the acceleration components
     */
    void calcAccelerationMagnitude();

    /**
     * Updates the Teensy state based on the latest data from the IMU, Tachometer,
     * and Ultrasonic Distance Sensor.
     * @param IMU_data The latest IMU data
     * @param tach_data The latest tachometer data
     * @param ultra_data The latest ultrasonic distance sensor data
     */
    void updateTState(IMUData IMU_data, float tachometer_rpm, UltrasonicData ultra_data);

    /**
     * Serializes the teensy state to prepare to send from teensy to jetson
     * @param buffer A pointer to the location in memory where the serialized data will be written
    */
    void serializeTeensyState(uint8_t *buffer);

    /**
     * Serializes the teensy state object and sends it over the provided serial
     * @param serial The serial port that you want to send the teensy state to
    */
    int sendOverSerial(HardwareSerial &serial);

    /**
     * Prints the formatted teensy state data to the serial monitor
     */
    void serialPrintTState();

    
};
#endif