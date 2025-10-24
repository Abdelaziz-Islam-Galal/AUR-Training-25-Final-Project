#include <Arduino.h>
#include "encoder.h"
#include "motor_driver.h"
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
}
extern float kp;
extern float ki;
extern float kd;
extern float pidError;
extern float pidPrevError;
extern float pidIntegral;

extern QueueHandle_t motorQueue;
extern float computePID(float targetDeg, float currentDeg);
extern void moveRobot(float v_linear, float omega, float magnitude);

const float WHEEL_BASE = 0.2;
const float wheel_radius = 0.03;
const float MAX_WHEEL_SPEED = 30.0;

typedef struct
{
    float angle;
    float magnitude;
} MotorCommand_t;

const TickType_t CONTROL_DELAY = pdMS_TO_TICKS(50);

void MotorInit();

// void setTargetPosition(float x, float y);
// void updateCurrentPosition(float x, float y, float theta);

void MotorTask(void *pvParameters)
{
    MotorInit();
    EncoderInit();
    Serial.println("MotorTask started");

    MotorCommand_t receivedCmd;
    float targetAngle = 0.0;
    float v_linear = 0.0;
    float omega = 0.0;
    float currentAngle = 0.0;

    for (;;)
    {
        BaseType_t status = xQueueReceive(motorQueue, &receivedCmd, portMAX_DELAY);

        // currentAngle = getCurrentAngle();

        if (status == pdPASS)
        {

            targetAngle = receivedCmd.angle;
            float magnitude = receivedCmd.magnitude;

            updateEncoders();
            float currentAngle = getCurrentAngle();

            omega = computePID(targetAngle, currentAngle);
            v_linear = magnitude * MAX_WHEEL_SPEED;

            moveRobot(v_linear, omega, magnitude);

            Serial.print("target:");
            Serial.println(targetAngle);
            Serial.print(" current:");
            Serial.println(currentAngle);
        }
        /*currentAngle += 5.0;
        if (currentAngle > 360.0)
            currentAngle -= 360.0;
*/
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void vOtherFunction(void *pvParameters)
{
    BaseType_t xReturned;
    TaskHandle_t xHandle = NULL;

    xReturned = xTaskCreate(
        MotorTask,
        "motor",
        4096,
        NULL,
        1,
        NULL);

    if (xReturned == pdPASS)
    {
        Serial.println("MotorTask created successfully!");
    }
    else
    {
        Serial.println("MotorTask creation failed!");
    }
    vTaskDelete(NULL);
}
