#include "teensy_control.h"

bool readControlSignalFromSerial(HardwareSerial &serial, TeensyControl &raw_control) {
    static enum { WAIT_HEADER1, WAIT_HEADER2, WAIT_PAYLOAD, WAIT_CHECKSUM } state = WAIT_HEADER1;
    static uint8_t buffer[CONTROL_SIZE];
    static uint8_t checksum = 0;
    static size_t index = 0;

    while (serial.available()) {
        uint8_t byte = serial.read();

        switch (state) {
            // wait for the first header byte. if its recieved, set the state to wait for the second byte
            case WAIT_HEADER1:
                if (byte == HEADER[0]) state = WAIT_HEADER2;
                break;

            // wait for the second byte. if its recieved set the state to wait for the payload
            case WAIT_HEADER2:
                if (byte == HEADER[1]) {
                    state = WAIT_PAYLOAD;
                    index = 0;
                    checksum = 0;
                } else {
                    state = WAIT_PAYLOAD;
                }
                break;
            
            // if we are in payload state, read a byte into the buffer and update the checksum.
            // if we've recieved the right number of payload bytes, set the state to wait for the checksum
            case WAIT_PAYLOAD:
                buffer[index++] = byte;
                checksum ^= byte;
                if (index >= CONTROL_SIZE) state = WAIT_CHECKSUM;
                break;

            // read the packet's checksum byte and compare it to the computed checksum.
            // if they are equal, copy the buffer into the control object then reset the state to start and return true
            // if they aren't, just reset the state to start
            case WAIT_CHECKSUM:
                if (checksum == byte) {
                    memcpy(&raw_control, buffer, CONTROL_SIZE);
                    state = WAIT_HEADER1;
                    return true;
                } else {
                    state = WAIT_HEADER1;
                }
                break;
        }
    }

    return false;   // We haven't received a full packet yet
}

void printControlSignal(const TeensyControl &my_control, Stream &serial) {
    serial.print("Control signal: [speed command: ");
    serial.print(my_control.speed_command);
    serial.print("m/s, steering command: ");
    serial.print(my_control.steering_command);
    serial.println("rad]");
}

double PIController::update(float speed_command, float measured_speed) {
    double dt = (micros() - t_prev_) / 100000;
    double error = speed_command - measured_speed;

    if (t_prev_ == 0) {
        integral_ = 0;
    } else {
        integral_ += dt * error;
    }

    t_prev_ = micros();
    
    double control_signal = kp_ * error + ki_* integral_;
    double throttle = control_signal + throttle_neutral;

    // Limit throttle to motor limits
    if (throttle > throttle_max) {
        throttle = throttle_max;
    }
    if (throttle < throttle_min) {
        throttle = throttle_min;
    }

    return throttle;
}

void PIController::reset() {
    integral_ = 0;
    t_prev_ = 0;
}

float SteeringController::steering_angle_to_servo(float steering_angle) {
    float steering_angle_deg = steering_angle * rad_2_deg;
    float servo_angle = steering_gain * steering_angle_deg + steering_neutral;

    return servo_angle;
}


