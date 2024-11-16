/****************************************************************************
Copyright 2021 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
****************************************************************************/

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include <Arduino.h>
#include <Bluepad32.h>
#include <Arduino_APDS9960.h>

#include <ESP32Servo.h>
#include <ESP32SharpIR.h>
#include <QTRSensors.h>

//SERVO
Servo myServo;
int pos = 1;


// Color Sensor
#define APDS9960_INT 0
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ 100000

TwoWire I2C_0 = TwoWire(0);
APDS9960 sensor = APDS9960(I2C_0, APDS9960_INT);

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

QTRSensors qtr;
uint16_t sensors[5];

int r, g, b, a, colorOne, colorTwo;

ESP32SharpIR frontDistanceSensor(ESP32SharpIR::GP2Y0A21YK0F, 39);
ESP32SharpIR leftDistanceSensor(ESP32SharpIR::GP2Y0A21YK0F, 36);
ESP32SharpIR rightDistanceSensor(ESP32SharpIR::GP2Y0A21YK0F, 34); 

// This callback gets called any time a new gamepad is connected.
void onConnectedGamepad(GamepadPtr gp)
{
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    {
        if (myGamepads[i] == nullptr)
        {
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
}

void onDisconnectedGamepad(GamepadPtr gp)
{
    bool foundGamepad = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    {
        if (myGamepads[i] == gp)
        {
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }
}

void color_sensor_read(){
    //ON BUTTON A PRESSED
    sensor.readColor(r, g, b, a);

    Serial.print("RED: ");
    Serial.println(r);
    Serial.print("GREEN: ");
    Serial.println(g);
    Serial.print("BLUE: ");
    Serial.println(b);
    Serial.print("AMBIENT: ");
    Serial.println(a);

    if(r > b && r > g)
    {
        Serial.println("Color is red.");
        colorOne = 1;
    } else if(g > r && g > b) {
        Serial.println("Color is green.");
        colorOne = 2;
    } else if(b > r && b > g) {
        Serial.println("Color is blue.");
        colorOne = 3;
    } else {
        Serial.println("Hmmm...");
    }
    // END BUTTON A PRESS
    delay (1000);
}

void color_sensor_check() 
{
    // ON BUTTON B PRESS
    int r2, g2, b2, a2;
    sensor.readColor(r2, g2, b2, a2);

    Serial.print("CHECK RED: ");
    Serial.println(r2);
    Serial.print("CHECK GREEN: ");
    Serial.println(g2);
    Serial.print("CHECK BLUE: ");
    Serial.println(b2);
    Serial.print("CHECK AMBIENT: ");
    Serial.println(a2);

    if(r2 > b2 && r2 > g2)
    {
        Serial.println("I am detecting... red.");
        colorTwo = 1;
    } else if(g2 > r2 && g2 > b2) {
        Serial.println("I am detecting... green.");
        colorTwo = 2;
    } else if(b2 > r2 && b2 > g2) {
        Serial.println("I am detecting... blue.");
        colorTwo = 3;
    } else {
        Serial.println("Hmmm...");
    }

    if(colorOne == colorTwo)
    {
        analogWrite(13, 0);
        analogWrite(25, 0);
        Serial.print("Same color.");
        delay(500);
        for(int i = 0; i < 4; i++)
        {
            digitalWrite(2, 1);
            delay(500);
            digitalWrite(2, 0);
            delay(500);
        }
    } else {
        analogWrite(13, 150);
        analogWrite(25, 150);
        digitalWrite(12, 0);
        digitalWrite(26, 0);
        digitalWrite(14, 1);
        digitalWrite(27, 1);
    }
    // END BUTTON B PRESS
}

void line_sensor_calibration()
{
    for(uint8_t i = 0; i <250; i++)
    {
        Serial.println("Calibrating...");
        qtr.calibrate();
        
        analogWrite(13, 100);
        analogWrite(25, 100);

        digitalWrite(12, 0);
        digitalWrite(26, 0);
        digitalWrite(14, 1);
        digitalWrite(27, 1);

        delay(20);

        analogWrite(13, 100);
        analogWrite(25, 100);

        digitalWrite(12, 1);
        digitalWrite(26, 1);
        digitalWrite(14, 0);
        digitalWrite(27, 0);

        delay(20);
    }
    analogWrite(13, 0);
    analogWrite(25, 0);
}

void line_sensor_follow()
{
    // Serial.println("yButton has been pressed");
    int where = qtr.readLineBlack(sensors);
    Serial.print("line sensor value: ");
    Serial.println(where);
    
    uint16_t s0 = sensors[0];
    uint16_t s1 = sensors[1];
    uint16_t s2 = sensors[2];
    uint16_t s3 = sensors[3];
    uint16_t s4 = sensors[4];

    Serial.print("Sensor 1: "); Serial.println(sensors[0]);
    Serial.print("Sensor 2: "); Serial.println(sensors[1]);
    Serial.print("Sensor 3: "); Serial.println(sensors[2]);
    Serial.print("Sensor 4: "); Serial.println(sensors[3]);
    Serial.print("Sensor 5: "); Serial.println(sensors[4]);
    
    if((s0 > 700) || (s1 > 700)) // turn right
    {
        
        analogWrite(13,80);
        analogWrite(25,90);

        digitalWrite(12, 0);
        digitalWrite(26, 0);
        digitalWrite(14, 1);
        digitalWrite(27, 1);
        //Serial.println("turn right");
    } else if((s2 > 700) || (s3 > 700) || (s4 > 700)) { // turn left
        analogWrite(13,100);
        analogWrite(25,90);

        digitalWrite(12, 0);
        digitalWrite(26, 0);
        digitalWrite(14, 1);
        digitalWrite(27, 1);
        //Serial.println("turn left");
    } else if(s2 == 0)
    {
        // reverse
        analogWrite(13, 80);
        analogWrite(25, 80);

        digitalWrite(12, 1);
        digitalWrite(26, 1);
        digitalWrite(14, 0);
        digitalWrite(27, 0);
    }
    delay(50);
}

void distance_sensor_run()
{
    float disFront = frontDistanceSensor.getDistanceFloat();
    float disLeft = leftDistanceSensor.getDistanceFloat();
    float disRight = rightDistanceSensor.getDistanceFloat();

    if(disFront < 15)
    {
        if(disRight > 15)
        {
            // turn right (back up then turn right)
            analogWrite(13, 100);
            analogWrite(25, 100);

            digitalWrite(12, 1);
            digitalWrite(26, 1);
            digitalWrite(14, 0);
            digitalWrite(27, 0);

            delay(200);

            analogWrite(13,80);
            analogWrite(25,255);

            digitalWrite(12, 0);
            digitalWrite(26, 0);
            digitalWrite(14, 1);
            digitalWrite(27, 1);
        } else {
            // turn left (back up then turn left)
            analogWrite(13, 100);
            analogWrite(25, 100);

            digitalWrite(12, 1);
            digitalWrite(26, 1);
            digitalWrite(14, 0);
            digitalWrite(27, 0);

            delay(200);

            analogWrite(13, 255);
            analogWrite(25,80);

            digitalWrite(12, 0);
            digitalWrite(26, 0);
            digitalWrite(14, 1);
            digitalWrite(27, 1);
        }
    } else {
        // go forward
        analogWrite(13, 100);
        analogWrite(25, 100);

        digitalWrite(12, 0);
        digitalWrite(26, 0);
        digitalWrite(14, 1);
        digitalWrite(27, 1);
    }
}

// Arduino setup function. Runs in CPU 1
void setup()
{
//SERVO SETUP
        myServo.attach(15);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // TODO: Write your setup code here
    I2C_0.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    sensor.setInterruptPin(APDS9960_INT);
    sensor.begin();

    Serial.begin(115200);

    pinMode(2, OUTPUT);

    pinMode(13, OUTPUT);
    pinMode(25, OUTPUT);

    pinMode(12, OUTPUT);
    pinMode(14, OUTPUT);
    pinMode(27, OUTPUT);
    pinMode(26, OUTPUT);

    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]) {33, 32, 15, 35, 4}, 5);    // Removed 0 
    // calibration will be a button in the loop()

    frontDistanceSensor.setFilterRate(1.0f);
    leftDistanceSensor.setFilterRate(1.0f);
    rightDistanceSensor.setFilterRate(1.0f);
}

// Arduino loop function. Runs in CPU 1
void loop()
{
    BP32.update();

    // line_sensor_calibration();
    // while(1) { line_sensor_follow(); }

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    {
        GamepadPtr myGamepad = myGamepads[i];
        if (myGamepad && myGamepad->isConnected())
        {
            //TODO: Write your controller code here
            Serial.print("controller connected...");

            // COLOR SENSING CODE BEGINS HERE (BUTTON A COLLECTS GOAL COLOR AND BUTTON B DETECTS IF COLOR MATCHES)
            while (!sensor.colorAvailable())
            {
                delay(5);
                Serial.print("no color available...");
            }

            bool A = myGamepad->b();
            // Serial.print(A);
            if(A)
            {
               color_sensor_read(); 
            }

            bool B = myGamepad->a();
            if(B)
            {
                color_sensor_check();
            }
            // END COLOR SENSOR CODE

//Servo Intake Code
     // Servo CODE BEGINS HERE (PIN 35)
            int RY = myGamepad->axisY();
            Serial.print("RY = ");
            Serial.println(RY);
            delay(100);
            if(RY < -50) {
                //turn servo output
                myServo.writeMicroseconds(1000);
            } else if (RY > 50) {
                //turn servo intake
                myServo.writeMicroseconds(2000);
            }


            } else {
                //don't move
                myServo.writeMicroseconds(1500);
            }
   
            // END Servo CODE

            // MOVEMENT CODE BEGINS HERE (PIN 13 is ENA, PIN 25 is ENB, 12/14/27/26 control their state)
            int X = myGamepad->axisX();
            int Y = myGamepad->axisY();
            Serial.print("X = ");
            Serial.println(X);
            Serial.print("Y = ");
            Serial.println(Y);
            delay(100);
            if(Y < -50) {
                //turn both motors on to move forward
                analogWrite(13, 255);
                analogWrite(25, 255);

                digitalWrite(12, 0);
                digitalWrite(26, 0);
                digitalWrite(14, 1);
                digitalWrite(27, 1);
            } else if (Y > 50) {
                //turn both motors on backward
                analogWrite(13, 255);
                analogWrite(25, 255);

                digitalWrite(12, 1);
                digitalWrite(26, 1);
                digitalWrite(14, 0);
                digitalWrite(27, 0);
            } else if(X > 50) {
                //turn left motor on faster than the right one (to turn right)
                analogWrite(13,100);
                analogWrite(25,255);

                digitalWrite(12, 0);
                digitalWrite(26, 0);
                digitalWrite(14, 1);
                digitalWrite(27, 1);
            } else if (X < -50)
            {
                //turn right motor on faster than the left one (to turn left)
                analogWrite(13, 255);
                analogWrite(25,100);

                digitalWrite(12, 0);
                digitalWrite(26, 0);
                digitalWrite(14, 1);
                digitalWrite(27, 1);
            } else { 
                //don't move
                analogWrite(13,0);
                analogWrite(25,0);
            }
            // END MOVEMENT CODE


            // BEGIN LINE SENSOR CODE
                // have a calibration button then a line follow button
            bool xButton = myGamepad->x();
            bool yButton = myGamepad->y();
            if(xButton) // calibration
            {
                line_sensor_calibration();
            }
            if(yButton) // follow the line
            {
                line_sensor_follow();
            }
            // END LINE SENSOR CODE


            // BEGIN DISTANCE SENSOR CODE
            bool leftShoulder = myGamepad->l1();
            if(leftShoulder)
            {
                distance_sensor_run();
            }
        }
            // END DISTANCE SENSOR CODE


            // BEGIN INTAKE SERVO CODE

            // END INTAKE SERVO CODE

    // TODO: Write your periodic code here

    vTaskDelay(1);

    }
}