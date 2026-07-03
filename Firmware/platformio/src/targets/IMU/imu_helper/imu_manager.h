#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

// IMU pins
#ifndef IMU_SDA
#define IMU_SDA 8
#endif

#ifndef IMU_SCL
#define IMU_SCL 9
#endif

#ifndef IMU_RST
#define IMU_RST 4
#endif

// How many samples to average
#ifndef IMU_AVG_SAMPLES
#define IMU_AVG_SAMPLES 50
#endif

// Struct to hold a single averaged IMU reading
struct IMUData {
    float accX, accY, accZ;
    float gyrX, gyrY, gyrZ;
    float linX, linY, linZ;
    float qi, qj, qk, qr;
    uint8_t quatAcc;
};

class IMUManager {
public:
    IMUManager();
    bool begin();
    bool update();                 // call in loop; returns true when data ready
    IMUData getData() const;

private:
    BNO08x imu;
    uint16_t sampleCount;
    IMUData accum;                 // accumulator for averaging
    IMUData latest;                // latest averaged result
};
