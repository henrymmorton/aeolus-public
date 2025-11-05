#include "sensors.h"
#include "teensy_pins.h"
#include "tstate.h"
#include "teensy_control.h"
#include <Servo.h>
#include <IntervalTimer.h>
#include <IRremote.hpp>

#define IR_RECEIVE_PIN 2
#define TWO_CODE 0xE718FF00

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
bool new_control;
TeensyControl controlSignal;
Servo myMotor;
Servo mySteering;
PIController myPIController(Pd, Pi);
SteeringController mySteeringController;

// Create the global throttle value
float throttle = 0;
float set_speed = 2.3;

// Create the global steering values
float steering_command;

// Create some timers
int buffer_size = 0;
unsigned long sync_time = 0;
unsigned long print_timer = 0;

void setup() {
  // Start serial communication
  //Serial.begin(9600);

  // Start the motor
  myMotor.attach(MOTOR_PIN, 1000, 2000);

  // Start the steering servo
  mySteering.attach(SERVO_PIN, 1000, 2000);

  // Wait for the motor ESC to arm
  delay(3000);

  // Start serial communication with the Jetson
  Serial1.begin(115200);
  while (!Serial1) {
  }

  Serial.println("Waiting for handshake signal...");

  // Start IR receiver
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  /*
  // Wait for IR signal
  while (true) {
    if (IrReceiver.decode()) {
      uint32_t ir_code = IrReceiver.decodedIRData.decodedRawData;
      Serial.print("Received IR code: 0x");
      Serial.println(ir_code, HEX);

      if (ir_code == TWO_CODE) {
        // Send signal/message over Serial1
        Serial.print("Starting jetson");
        Serial1.println("Go");
        // Stop IR receiver if you no longer need it
        IrReceiver.end();  // Optional
        break;  // Exit the wait loop
      }

      IrReceiver.resume();
    }
  }
  delay(1000);
  */

  // Wait untill the timestamp handshake with the Jetson has been performed to start main loop
  while(true) {
    if (Serial1.available()) {
      String sync_message = Serial1.readStringUntil('\n');
      sync_message.trim();

      if (sync_message == "SYNC") {
        sync_time = micros();
        Serial1.print("SYNC_REPLY ");
        Serial1.println(sync_time);
        // Serial.print("recieved sync message, replying with time");
        break;
      }
    }
  }

  while (Serial1.available()) Serial1.read();

  Serial.print("Cleared buffer?: ");
  Serial.println(Serial1.available());
  
  delay(500);
  
  // Initialize the sensor objects
  my_IMU.initIMU();
  my_tach.initTachometer();

  // Start the tachometer measurement timer
  tach_timer.begin(read_tachometer_wrapper, 1000);
}

void loop() {
  // Read the control signals from the jetson
  new_control = readControlSignalFromSerial(Serial1, controlSignal);

  // Measure IMU values
  my_IMU.updateIMUData();
  c_imu_data = my_IMU.current_imu_data;

  // Update the teensy state
  c_teensy_state.updateTState(c_imu_data, latest_tach_reading, c_ultra_data);

  // Call the PI and steering controllers
  throttle = myPIController.update(controlSignal.speed_command, c_teensy_state.velocity_mag);
  steering_command = mySteeringController.steering_angle_to_servo(controlSignal.steering_command);

  // Send the control signal to the motor
  myMotor.writeMicroseconds(throttle);

  // Send the control signal to the steering servo
  mySteering.write(steering_command);

  // If desired, print out the teensy state every second
  if (micros() - print_timer > 1000000) {
    if (new_control) {
      printControlSignal(controlSignal, Serial);
    }
    //c_teensy_state.serialPrintTState();
    Serial.println(latest_tach_reading);
    print_timer = micros();
  }

  // TODO: Figure out why the timer based tach implementation is not updatomg
  
  // Send the teensy state object to the jetson
  buffer_size = c_teensy_state.sendOverSerial(Serial1);

  delay(10);
}

// A wrapper that updates the tachometer for use with the tachometer timer
void read_tachometer_wrapper() {
  my_tach.updateTachometer();
  latest_tach_reading = my_tach.rpm;
}