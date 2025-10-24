#include <Arduino.h>
#include "motor_driver.h"
#include "encoder.h"

const unsigned long dt_ms = 50;
const float WHEEL_BASE = 0.2;
const float wheel_radius = 0.03;
const float MAX_WHEEL_SPEED = 50.0;

float kp = 1.0;
float ki = 0.0;
float kd = 0.1;

static float targetX = 0.0, targetY = 0.0, targetZ = 0.0, currentX = 0.0, currentY = 0.0, currentTheta = 0.0;

float pidError = 0.0, pidPrevError = 0.0, pidIntegral = 0.0, pidOutput = 0.0;

float targetAngle = 0.0;


bool timingfunc(unsigned long interval_ms)
{
    static unsigned long prevTime = 0;
    unsigned long now = millis();
    if (now - prevTime >= interval_ms)
    {
        prevTime = now;
        return true;
    }
    return false;
}

void MotorInit();
void setMotor(int IN1, int IN2, int channel, int PWMVALUE);
float computePID(float targetDeg, float currentDeg);
void moveRobot(float v_linear, float omega, float magnitude);
int linearSpeedtoPWM(float wheelLinearSpeed);
void setTargetPosition(float x, float y);
void updateCurrentPosition(float x, float y, float theta);
bool timingfunc(unsigned long interval_ms);

void MotorInit()
{
    pinMode(L_IN1, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(R_IN1, OUTPUT);
    pinMode(R_IN2, OUTPUT);
    ledcSetup(L_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(R_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(L_PWM, L_CHANNEL);
    ledcAttachPin(R_PWM, R_CHANNEL);
    ledcWrite(L_CHANNEL, 0);
    ledcWrite(R_CHANNEL, 0);
}

int linearSpeedtoPWM(float wheelLinearSpeed)
{
    float wheelAngularSpeed = wheelLinearSpeed / wheel_radius;
    float ratio = wheelAngularSpeed / MAX_WHEEL_SPEED;

    ratio = constrain(ratio, -1.0, 1.0);

    int pwm = (int)(ratio * 255.0);
    return pwm;
}

float computePID(float targetDeg, float currentDeg)
{
    float errorDeg = targetDeg - currentDeg;
    while (errorDeg > 180.0)
        errorDeg -= 360.0;
    while (errorDeg < -180.0)
        errorDeg += 360.0;
    pidError = errorDeg;
    float dt = dt_ms / 1000.0;
    pidIntegral += pidError * dt;
    float derivative = (pidError - pidPrevError) / dt;
    pidPrevError = pidError;
    float outDegPerSec = (kp * pidError) + (ki * pidIntegral) + (kd * derivative);
    float outRadPerSec = outDegPerSec * (PI / 180.0);
    float maxOmegaFromWheels = (2.0 * MAX_WHEEL_SPEED) / WHEEL_BASE;
    float MAX_OMEGA = 2 * PI;
    if (outRadPerSec > MAX_OMEGA)
        outRadPerSec = MAX_OMEGA;
    if (outRadPerSec < -MAX_OMEGA)
        outRadPerSec = -MAX_OMEGA;
    pidOutput = outRadPerSec;
    return pidOutput;
}

void setMotor(int IN1, int IN2, int channel, int pwmValue)
{
    if (pwmValue > 255)
        pwmValue = 255;
    if (pwmValue < -255)
        pwmValue = -255;
    if (pwmValue >= 0)
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        ledcWrite(channel, pwmValue);
    }
    else
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        ledcWrite(channel, -pwmValue);
    }
}

void moveRobot(float v_linear, float omega, float magnitude)
{

    float v_left = v_linear - (WHEEL_BASE / 2.0) * omega;
    float v_right = v_linear + (WHEEL_BASE / 2.0) * omega;
    int pwmLeft = linearSpeedtoPWM(v_left);
    int pwmRight = linearSpeedtoPWM(v_right);
    setMotor(L_IN1, L_IN2, L_CHANNEL, pwmLeft);
    setMotor(R_IN1, R_IN2, R_CHANNEL, pwmRight);
}

void setup()
{
    Serial.begin(9600);
    MotorInit();
    EncoderInit();
}

void loop()
{

    float magnitude = 0.3;
    targetAngle = 180.0;

    if (timingfunc(50))
    {
        updateEncoders();
        float currentAngle = getCurrentAngle();
        float v_linear = magnitude * MAX_WHEEL_SPEED;
        float omega = computePID(targetAngle, currentAngle);
        moveRobot(v_linear, omega, magnitude);

        Serial.print("Tdeg=");
        Serial.print(targetAngle);
        Serial.print(" Cdeg=");
        Serial.print(currentAngle);
        Serial.print(" omega=");
        Serial.print(omega, 3);
        Serial.print(" pwmL=");
        Serial.println((int)linearSpeedtoPWM((v_linear - (WHEEL_BASE / 2.0) * omega)));
        Serial.print(" pwmR=");
        Serial.println((int)linearSpeedtoPWM((v_linear + (WHEEL_BASE / 2.0) * omega)));
    }
}
/*
void setTargetPosition(float x, float y)
{
    targetX = x;
    targetY = y;

    targetAngle = atan2(targetY - currentY, targetX - currentX) * 180.0 / PI;
    if (targetAngle < 0)
        targetAngle += 360.0;
}

void updateCurrentPosition(float x, float y, float theta)
{
    currentX = x;
    currentY = y;
    currentTheta = theta;
    currentAngle = theta * 180.0 / PI;
}



*/
/*
#include <Arduino.h>

#define L_IN1 12
#define L_IN2 13
#define L_PWM 25
#define L_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

void setMotor(int IN1, int IN2, int channel, int pwmValue);

void setup()
{
    pinMode(L_IN1, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    ledcSetup(L_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(L_PWM, L_CHANNEL);
    Serial.begin(9600);
    Serial.println("Motor Test Start");
}

void loop()
{
    Serial.println("Forward");
    setMotor(L_IN1, L_IN2, L_CHANNEL, 200);
    delay(3000);

    Serial.println("Reverse");
    setMotor(L_IN1, L_IN2, L_CHANNEL, -200);
    delay(3000);
}

void setMotor(int IN1, int IN2, int channel, int pwmValue)
{
    pwmValue = constrain(pwmValue, -255, 255);

    if (pwmValue >= 0)
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        ledcWrite(channel, pwmValue);
    }
    else
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        ledcWrite(channel, -pwmValue);
    }
}

*/