#include "sensors.h"
#include "teensy_pins.h"
#include "tstate.h"
#include "teensy_control.h"
#include <Servo.h>
#include <IntervalTimer.h>


// Create the data objects
IMUData c_imu_data;
UltrasonicData c_ultra_data;
TeensyState c_teensy_state;

// Create the sensor objects
Tachometer my_tach(TACHOMETER_PIN);
IMU my_IMU;
UltraSonicDistanceSensor my_ultra(ULTRA_TRIG_PIN, ULTRA_ECHO_PIN);

// Create an interval timer to run the tachometer independent of the main loop
IntervalTimer tach_timer;

// Create a volatile global variable to store the tachometer reads
volatile float latest_tach_reading = 0;

// Create the control objects
TeensyControl controlSignal;
Servo myMotor;
Servo mySteering;

PIController myPIcontroller(*, 0.1);

// Create some timers
int buffer_size = 0;
unsigned long sync_time = 0;
unsigned long print_timer = 0;

float velocity_mag = 0;



void setup() {
  // Start serial
  Serial.begin(9600);

  // Start the motor
  myMotor.attach(MOTOR_PIN, 1000, 2000);
  myMotor.writeMicroseconds(1500);

  // Wait for the ESC to arm
  delay(3000);

  // Start the steering servo
  mySteering.attach(SERVO_PIN, 1000, 2000);

  // Initialize the sensor objects
  my_IMU.initIMU();
  my_tach.initTachometer();

  // Start the tachometer measurement process
  tach_timer.begin(read_tachometer_wrapper, 1000);
}

void loop() {
    // Measure the IMU
    my_IMU.updateIMUData();
    c_imu_data = my_IMU.current_imu_data;

    // Update the teensy state
    c_teensy_state.updateTState(c_imu_data, latest_tach_reading, c_ultra_data);
    
    // Call the PI controller
    velocity_mag = c_teensy_state.velocity_mag;
    throttle = myPIcontroller.update(set_speed, velocity_mag);

    // Send the control signal to the motor
    myMotor.writeMicroseconds(throttle);

    delay(100);
}

void read_tachometer_wrapper() {
  my_tach.updateTachometer();
  latest_tach_reading = my_tach.rpm;
}