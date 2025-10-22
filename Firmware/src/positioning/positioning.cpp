#include "positioning.h"
#include <esp32-hal.h>
#include <Wire.h>

#include "driver/pulse_cnt.h"


Encoder::Encoder(const gpio_num_t pin_a, const gpio_num_t pin_b) {
    const pcnt_chan_config_t chan_config_0(pin_a, pin_b);
    const pcnt_chan_config_t chan_config_1(pin_b, pin_a);
    constexpr pcnt_unit_config_t unit_config = {
            .low_limit = INT32_MIN,
            .high_limit = INT32_MAX,
            .intr_priority = 0,
            .flags = {
                .accum_count = false
            }
        };
    pcnt_channel_handle_t channel_0 = nullptr;
    pcnt_channel_handle_t channel_1 = nullptr;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &unit));
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_config_0, &channel_0));
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_config_1, &channel_1));
    pcnt_channel_set_edge_action(channel_0, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(channel_0, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_channel_set_edge_action(channel_1, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(channel_1, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
}


void Positioning::setup() {
}

Positioning::Positioning(const gpio_num_t encoder_0_pin_a, const gpio_num_t encoder_0_pin_b,
                         const gpio_num_t encoder_1_pin_a, const gpio_num_t encoder_1_pin_b
) :
    prev_time(millis()),
    _encoders{
        Encoder(encoder_0_pin_a, encoder_0_pin_b),
        Encoder(encoder_1_pin_a, encoder_1_pin_b)
    } {
    // MPU6050 setup
    Wire.begin();
    mpu.initialize();
    mpu.dmpInitialize();

    // TODO: calibrate offsets
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

    // mpu calibration
    mpu.CalibrateAccel(6); // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
}

void Positioning::update() {
    Quaternion q;
    // VectorInt16 aa;
    // VectorInt16 aaReal;
    // VectorInt16 aaWorld;
    VectorFloat gravity;
    uint8_t FIFOBuffer[64];

    // double dt = (millis() - prev_time) / 1000.0;

    mpu.dmpGetCurrentFIFOPacket(FIFOBuffer);
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);

    // mpu.dmpGetAccel(&aa, FIFOBuffer);
    // mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    // mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    const double dl = _encoders[0].count() * (2.0 * M_PI * WHEEL_RADIUS) / (ENCODER_RESOLUTION);
    const double dr = _encoders[1].count() * (2.0 * M_PI * WHEEL_RADIUS) / (ENCODER_RESOLUTION);

    for (auto& encoder : _encoders) {
        encoder.reset();
    }

    const double d = (dl + dr) / 2.0;

    float ypr[3];
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    _position.x = d * cos(static_cast<const double>(ypr[0]));
    _position.y = d * sin(static_cast<const double>(ypr[0]));
    _position.yaw = ypr[0];

    // TODO: kalman filter
}
