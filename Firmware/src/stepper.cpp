#include <Arduino.h>
#define DIR 18
#define STP 19
#define PB1 13
#define PB2 12
#define PB3 14
#define PB4 27

void setup()
{
    Serial.begin(115200);

    pinMode(DIR, OUTPUT);
    pinMode(STP, OUTPUT);

    pinMode(PB1, INPUT_PULLUP);
    pinMode(PB2, INPUT_PULLUP);
    pinMode(PB3, INPUT_PULLUP);
    pinMode(PB4, INPUT_PULLUP);
}

void loop()
{
    bool a = !digitalRead(PB1);
    bool b = !digitalRead(PB2);
    bool c = !digitalRead(PB3);
    bool d = !digitalRead(PB4);

    int steps = 0;

    if (a)
    {
        Serial.println("Button 1 pressed");
        digitalWrite(DIR, LOW);
        steps = 200;
    }
    else if (b)
    {
        Serial.println("Button 2 pressed");
        digitalWrite(DIR, HIGH);
        steps = 200;
    }
    else if (c)
    {
        Serial.println("Button 3 pressed");
        digitalWrite(DIR, LOW);
        steps = 400;
    }
    else if (d)
    {
        Serial.println("Button 4 pressed");
        digitalWrite(DIR, HIGH);
        steps = 400;
    }

    if (steps > 0)
    {
        for (int i = 0; i < steps; i++)
        {
            digitalWrite(STP, HIGH);
            delayMicroseconds(1000);
            digitalWrite(STP, LOW);
            delayMicroseconds(1000);
        }
    }
}
