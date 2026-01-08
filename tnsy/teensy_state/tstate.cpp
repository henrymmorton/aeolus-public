#include "tstate.h"

void TeensyState::updateTState(IMUData IMU_data, float tachometer_rpm, UltrasonicData ultra_data) {

    time = micros();

    x_acc = IMU_data.x_acc;
    y_acc = IMU_data.y_acc;
    z_acc = IMU_data.z_acc;
    yaw_rate = IMU_data.yaw_rate;
    pitch_rate = IMU_data.pitch_rate;
    roll_rate = IMU_data.roll_rate;
    
    yaw = IMU_data.yaw;
    pitch = IMU_data.pitch;
    roll = IMU_data.roll;

    new_accel_data = IMU_data.new_accel_data;
    new_euler_data = IMU_data.new_euler_data;

    rpm = tachometer_rpm;

    obstc_dist = ultra_data.ob_dist;
    obstc_detected = ultra_data.detected;

    calcVelocity();
    calcAccelerationMagnitude();
}

void TeensyState::calcVelocity() {
    velocity_mag = (((rpm / gear_ratio) * (2 * PI * wheel_radius)) / 60);
    vel_gx = sin(yaw) * velocity_mag;
    vel_gy = cos(yaw) * velocity_mag;
}

void TeensyState::calcAccelerationMagnitude() {
    acc_mag = sqrt(pow(x_acc, 2) + pow(y_acc, 2) + pow(z_acc, 2));
}

void TeensyState::serializeTeensyState(uint8_t *buffer) {
    size_t offset = 0;
    memcpy(buffer + offset, &time, sizeof(time));
    offset += sizeof(time);

    memcpy(buffer + offset, &x_acc, sizeof(x_acc));
    offset += sizeof(x_acc);

    memcpy(buffer + offset, &y_acc, sizeof(y_acc));
    offset += sizeof(y_acc);

    memcpy(buffer + offset, &z_acc, sizeof(z_acc));
    offset += sizeof(z_acc);

    memcpy(buffer + offset, &yaw_rate, sizeof(yaw_rate));
    offset += sizeof(yaw_rate);

    memcpy(buffer + offset, &pitch_rate, sizeof(pitch_rate));
    offset += sizeof(pitch_rate);

    memcpy(buffer + offset, &roll_rate, sizeof(roll_rate));
    offset += sizeof(roll_rate);

    memcpy(buffer + offset, &yaw, sizeof(yaw));
    offset += sizeof(yaw);

    memcpy(buffer + offset, &pitch, sizeof(pitch));
    offset += sizeof(pitch);

    memcpy(buffer + offset, &roll, sizeof(roll));
    offset += sizeof(roll);

    memcpy(buffer + offset, &rpm, sizeof(rpm));
    offset += sizeof(rpm);

    memcpy(buffer + offset, &obstc_dist, sizeof(obstc_dist));
    offset += sizeof(obstc_dist);

    memcpy(buffer + offset, &velocity_mag, sizeof(velocity_mag));
    offset += sizeof(velocity_mag);

    memcpy(buffer + offset, &vel_gx, sizeof(vel_gx));
    offset += sizeof(vel_gx);

    memcpy(buffer + offset, &vel_gy, sizeof(vel_gy));
    offset += sizeof(vel_gy);

    memcpy(buffer + offset, &acc_mag, sizeof(acc_mag));
    offset += sizeof(acc_mag);

    memcpy(buffer + offset, &obstc_detected, sizeof(obstc_detected));
    offset += sizeof(obstc_detected);
}

int TeensyState::sendOverSerial(HardwareSerial &serial) {
    // Calculate the sum sized of all the data members to be sent
    size_t buffer_size = sizeof(time) + sizeof(x_acc) + sizeof(y_acc) + sizeof(z_acc) + sizeof(yaw_rate) + sizeof(pitch_rate) + sizeof(roll_rate)
                         + sizeof(yaw) + sizeof(pitch) + sizeof(roll) + sizeof(rpm) + sizeof(obstc_dist)
                         + sizeof(velocity_mag) + sizeof(vel_gx) + sizeof(vel_gy) + sizeof(acc_mag) + sizeof(obstc_detected);

    

    // Allocate buffer memory to hold the data members
    uint8_t *buffer = new uint8_t[buffer_size];

    // Serialize the object
    serializeTeensyState(buffer);

    // Send over UART
    serial.write(buffer, buffer_size);

    // Deallocate the buffer memory
    delete[] buffer;

    return buffer_size;
}

void TeensyState::serialPrintTState() {
    Serial.print("Time Since Initaition: ");
    Serial.println(time/pow(10, 6));

    if (new_accel_data) {
        Serial.print(F("X Accel: "));
        Serial.print(x_acc, 3);
        Serial.print(F(" | Y Accel: "));
        Serial.print(y_acc, 3);
        Serial.print(F(" | Z Accel: "));
        Serial.print(z_acc, 3);
        Serial.print(" | Accel Magnitude: ");
        Serial.println(acc_mag, 3);

        Serial.print(F("Yaw Rate: "));
        Serial.print(yaw_rate, 3);
        Serial.print(F(" | Pitch Rate: "));
        Serial.print(pitch_rate,3);
        Serial.print(F(" | Roll Rate: "));
        Serial.println(roll_rate, 3);
    } else {
        Serial.println("No new acceleromter and gyro data");
    }

    if (new_euler_data) {
        Serial.print(F("Yaw: "));
        Serial.print(yaw, 3);
        Serial.print(F(" | Pitch: "));
        Serial.print(pitch, 3);
        Serial.print(F(" | Roll: "));
        Serial.println(roll, 3);
    } else {
        Serial.println("No new DMP data");
    }

    Serial.print("RPM: ");
    Serial.println(rpm);
    
    Serial.print("Velocity Magnitude: ");
    Serial.print(velocity_mag, 3);
    Serial.print(" | Velocity Global X Component: ");
    Serial.print(vel_gx, 3);
    Serial.print(" | Velocity Global Y Component: ");
    Serial.println(vel_gy, 3);

    Serial.print("Obstacle Detected?: ");
    if (obstc_detected) {
        Serial.print("Yes ");
        Serial.print("| Obstacle Distance: ");
        Serial.println(obstc_dist, 3);
    } else {
        Serial.println("No");
    }

    Serial.println("");
}