#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H
#include <Arduino.h>

#define L_IN1 25
#define L_IN2 26
#define L_PWM 27

#define R_IN1 32
#define R_IN2 33
#define R_PWM 12

#define L_CHANNEL 0
#define R_CHANNEL 1

#define PWM_FREQ 50000
#define PWM_RESOLUTION 8

void MotorInit();
void setMotor(int in1, int in2, int channel, int pwmValue);

#endif