#include <Arduino.h>
#define Spin 18
#define Cpin 12

void setup()
{
    Serial.begin(115200);
    pinMode(Spin, OUTPUT);
    pinMode(Cpin, INPUT);
    ledcSetup(0, 50, 16); // chanel 0 .freq50 .resolution 16 bits
    ledcAttachpin(Spin, 0);
}

void loop()
{
    int val = pulseIn(Cpin, HIGH, 25000);
    if (val > 0)
    {
        int angle = map(val, 500, 1500, 0, 180);
        int servoPulse = map(angle, 0, 180, 500, 2500);

        int duty = (servoPulse * 65535) / 20000;

        ledcWrite(0, duty);

        Serial.print("Angle: ");
        Serial.print(angle);
        Serial.print(" | Pulse: ");
        Serial.println(servoPulse);
    }

    // put your main code here, to run repeatedly:
}
