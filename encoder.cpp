#include "encoder.h"

volatile long leftTicks = 0;
volatile long rightTicks = 0;

// كل نبضة إنكودر تمثل زاوية معينة
const float TICKS_PER_REV = 600.0; // عدلها حسب نوع الإنكودر بتاعك
const float DEGREES_PER_TICK = 360.0 / TICKS_PER_REV;

// ==================== INTERRUPT FUNCTIONS ====================
void IRAM_ATTR onLeftAChange()
{
    int b = digitalRead(ENCODER_LEFT_B);
    if (b == HIGH)
        leftTicks++;
    else
        leftTicks--;
}

void IRAM_ATTR onRightAChange()
{
    int b = digitalRead(ENCODER_RIGHT_B);
    if (b == HIGH)
        rightTicks++;
    else
        rightTicks--;
}

// ==================== INITIALIZATION ====================
void EncoderInit()
{
    pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), onLeftAChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), onRightAChange, CHANGE);

    Serial.println(" Encoders initialized");
}

// ==================== UPDATE AND GET FUNCTIONS ====================
void updateEncoders()
{
    // ممكن في المستقبل نضيف فلترة أو حساب السرعة هنا
}

float getCurrentAngle()
{
    long avgTicks = (leftTicks + rightTicks) / 2;
    float angle = avgTicks * DEGREES_PER_TICK;

    // نحافظ على الزاوية بين 0 و 360
    while (angle >= 360.0)
        angle -= 360.0;
    while (angle < 0.0)
        angle += 360.0;

    return angle;
}