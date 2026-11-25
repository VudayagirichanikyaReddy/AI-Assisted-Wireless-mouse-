#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t MPU_ADDR = 0x68;
constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;

bool readMPU(
    int16_t &ax,
    int16_t &ay,
    int16_t &az,
    int16_t &gx,
    int16_t &gy,
    int16_t &gz
) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    uint8_t n = Wire.requestFrom(
        MPU_ADDR,
        (uint8_t)14,
        (uint8_t)true
    );

    if (n != 14 || Wire.available() < 14) {
        return false;
    }

    ax = (int16_t)((Wire.read() << 8) | Wire.read());
    ay = (int16_t)((Wire.read() << 8) | Wire.read());
    az = (int16_t)((Wire.read() << 8) | Wire.read());

    // Temperature
    Wire.read();
    Wire.read();

    gx = (int16_t)((Wire.read() << 8) | Wire.read());
    gy = (int16_t)((Wire.read() << 8) | Wire.read());
    gz = (int16_t)((Wire.read() << 8) | Wire.read());

    return true;
}

void setup() {
    Serial.begin(115200);
    delay(300);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    // Wake MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Accelerometer ±2g
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Gyroscope ±250 deg/s
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    Serial.println("AIR_MOUSE_WOKWI_READY");
}

void loop() {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    if (readMPU(ax, ay, az, gx, gy, gz)) {
        Serial.printf(
            "IMU,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f\n",
            ax / 16384.0f,
            ay / 16384.0f,
            az / 16384.0f,
            gx / 131.0f,
            gy / 131.0f,
            gz / 131.0f
        );
    } else {
        Serial.println("IMU,ERROR,ERROR,ERROR,ERROR,ERROR,ERROR");
    }

    delay(20);
}
