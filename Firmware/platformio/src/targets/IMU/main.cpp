#include <Arduino.h>
#include "targets/IMU/imu_helper/imu_manager.h"

IMUManager imu;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    if (!imu.begin()) {
        Serial.println("IMU init failed! Halting...");
        while (1);
    }
}

void loop() {
    if (imu.update()) {
        IMUData d = imu.getData();

        Serial.print("ACC[g] X:"); Serial.print(d.accX,3);
        Serial.print(" Y:"); Serial.print(d.accY,3);
        Serial.print(" Z:"); Serial.println(d.accZ,3);

        Serial.print("GYRO[dps] X:"); Serial.print(d.gyrX,3);
        Serial.print(" Y:"); Serial.print(d.gyrY,3);
        Serial.print(" Z:"); Serial.println(d.gyrZ,3);

        Serial.print("LIN[g] X:"); Serial.print(d.linX,3);
        Serial.print(" Y:"); Serial.print(d.linY,3);
        Serial.print(" Z:"); Serial.println(d.linZ,3);

        Serial.print("QUAT: (");
        Serial.print(d.qi,4); Serial.print(", ");
        Serial.print(d.qj,4); Serial.print(", ");
        Serial.print(d.qk,4); Serial.print(", ");
        Serial.print(d.qr,4);
        Serial.print(") acc:"); Serial.println(d.quatAcc);

        Serial.println("---");
    }
}
