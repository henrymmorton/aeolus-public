#include "sensors.h"

void UltrasonicData::processRawDistance(float raw_distance) {
    if (raw_distance >= 0 && raw_distance <= 400) {
        ob_dist = raw_distance;
    }

    if (ob_dist > 200 || ob_dist < 0) {
        detected = false;
    } else {
        detected = true;
    }
}

void UltrasonicData::printUltrasonicData() {
    Serial.print("Object Detected?: ");
    if (detected) {
        Serial.print("Yes");
    } else {
        Serial.print("No");
    }
    Serial.print(" | Distance: ");
    Serial.println(ob_dist);
}


