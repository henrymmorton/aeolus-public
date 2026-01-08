#include "sensors.h"  //include sensor header

void IMU::initIMU() {

    Wire.begin(); //Initates the wire libary and joins the I2C bus
    Wire.setClock(400000); //Sets the I2C clock frequency to 400kHz

    bool initialized = false;
    while (!initialized) {
        imu.begin(Wire, IMUI2C);  //Initialize the imu
        Serial.print(F("Initialization of the sensor returned: ")); //Print the imu status
        Serial.println(imu.statusString());
        if (imu.status != ICM_20948_Stat_Ok) { //If the imu returns an error, wait 0.5 seconds and try again
            Serial.println("Trying again...");
            delay(500);
        }
        else {
            initialized = true;
        }
    }

    Serial.println(F("Device connected!"));

    bool success = true; // Use success to show if the DMP configuration was successful

    success &= (imu.initializeDMP() == ICM_20948_Stat_Ok); //initialize the digital motion processor

    success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok); //Enable the 32-bit 6 axis quaternion output mode for the DMP

    success &= (imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set the ODR (output data rate, or sampling rate) to the maximum value

    // Enable the FIFO
    success &= (imu.enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (imu.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (imu.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (imu.resetFIFO() == ICM_20948_Stat_Ok);

    // Check success
    if (success)
    {
        Serial.println(F("DMP enabled!"));
    }
    else
    {
        Serial.println(F("Enable DMP failed!"));
        Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
        while (1)
        ; // Do nothing more
    }
}

IMUData& IMU::getIMUData() {
    return current_imu_data;
}

void IMU::updateIMUData() {
    if (imu.dataReady()) {
        imu.getAGMT();
        current_imu_data.x_acc = imu.accX() * mg2mps;
        current_imu_data.y_acc = imu.accY() * mg2mps;
        current_imu_data.z_acc = imu.accZ() * mg2mps;
        current_imu_data.yaw_rate = imu.gyrX();
        current_imu_data.pitch_rate = imu.gyrX();
        current_imu_data.roll = imu.gyrX();
        current_imu_data.new_accel_data = true;
    } else {
        current_imu_data.new_accel_data = false;
        // Serial.println("Waiting for normal data");
        // delay(50); //If no new data is available. wait 0.01s
    }

    icm_20948_DMP_data_t dmp_data;
    imu.readDMPdataFromFIFO(&dmp_data);

    if ((imu.status == ICM_20948_Stat_Ok) || (imu.status == ICM_20948_Stat_FIFOMoreDataAvail)) { // Was valid data available?
        if ((dmp_data.header & DMP_header_bitmap_Quat6) > 0) { // We have asked for GRV data so we should receive Quat6
            double q1 = ((double)dmp_data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
            double q2 = ((double)dmp_data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
            double q3 = ((double)dmp_data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

            // Convert the quaternions to Euler angles (roll, pitch, yaw)
            // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

            double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

            double q2sqr = q2 * q2;

            // roll (x-axis rotation)
            double t0 = +2.0 * (q0 * q1 + q2 * q3);
            double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
            current_imu_data.roll = atan2(t0, t1) * 180.0 / PI;

            // pitch (y-axis rotation)
            double t2 = +2.0 * (q0 * q2 - q3 * q1);
            t2 = t2 > 1.0 ? 1.0 : t2;
            t2 = t2 < -1.0 ? -1.0 : t2;
            current_imu_data.pitch = asin(t2) * 180.0 / PI;

            // yaw (z-axis rotation)
            double t3 = +2.0 * (q0 * q3 + q1 * q2);
            double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
            current_imu_data.yaw = atan2(t3, t4) * 180.0 / PI;

            current_imu_data.new_euler_data = true;
        }

    } else if (imu.status != ICM_20948_Stat_FIFOMoreDataAvail) {
        current_imu_data.new_euler_data = false;
        // Serial.println("Waiting for DMP data");
        // delay(10);  //If no new data is avaiable, wait 0.01s
    } else {
        Serial.println("Status error");
    }
}

void IMUData::serialPrintIMUData() {
    if (new_accel_data) {
        Serial.print(F("X Accel:"));
        Serial.print(x_acc, 3);
        Serial.print(F(" | Y Accel:"));
        Serial.print(y_acc, 1);
        Serial.print(F(" | Z Accel:"));
        Serial.print(z_acc, 1);
        Serial.print(F("| Yaw Rate:"));
        Serial.print(yaw_rate, 1);
        Serial.print(F(" | Pitch Rate:"));
        Serial.print(pitch_rate,1);
        Serial.print(F(" | Roll Rate:"));
    } else {
        Serial.print("No new acclerometer data");
    }
        
    if (new_euler_data) {
        Serial.print(roll_rate, 1);
        Serial.print(F(" | Yaw:"));
        Serial.print(yaw, 1);
        Serial.print(F(" | Pitch:"));
        Serial.print(pitch, 1);
        Serial.print(F(" | Roll:"));
        Serial.println(roll, 1);
    } else {
        Serial.print("No new euler data");
    }
}




