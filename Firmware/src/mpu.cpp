#include <Arduino.h>
#include <Wire.h>

constexpr auto mpu_adr = 0x68;

void init_mpu()
{
    Wire.begin();
    Wire.beginTransmission(mpu_adr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
}

void setup()
{
    Serial.begin(115200);
    init_mpu();
}

void loop()
{
    Wire.beginTransmission(mpu_adr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(mpu_adr, 14, true); // Read 14 bytes

    int16_t acc_x = Wire.read() << 8 | Wire.read();
    int16_t acc_y = Wire.read() << 8 | Wire.read();
    int16_t acc_z = Wire.read() << 8 | Wire.read();
    int16_t temp = Wire.read() << 8 | Wire.read();
    int16_t gyro_x = Wire.read() << 8 | Wire.read();
    int16_t gyro_y = Wire.read() << 8 | Wire.read();
    int16_t gyro_z = Wire.read() << 8 | Wire.read();

    Serial.print("Accel X = ");
    Serial.print(acc_x);
    Serial.print(" | Accel Y = ");
    Serial.print(acc_y);
    Serial.print(" | Accel Z = ");
    Serial.print(acc_z);
    Serial.print(" | Gyro X = ");
    Serial.print(gyro_x);
    Serial.print(" | Gyro Y = ");
    Serial.print(gyro_y);
    Serial.print(" | Gyro Z = ");
    Serial.println(gyro_z);

    delay(500);
}
