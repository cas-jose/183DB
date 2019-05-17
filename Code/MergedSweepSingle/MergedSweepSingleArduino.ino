//=======================From single.ino
/* This example shows how to get single-shot range
  measurements from the VL53L0X. The sensor can optionally be
  configured with different ranging profiles, as described in
  the VL53L0X API user manual, to get better performance for
  a certain application. This code is based on the four
  "SingleRanging" examples in the VL53L0X API.

  The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;


// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

//#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
//#define HIGH_ACCURACY

//============================
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//=========================From sweep.ino
/* Sweep
  by BARRAGAN <http://barraganstudio.com>
  This example code is in the public domain.

  modified 8 Nov 2013
  by Scott Fitzgerald
  http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>


//LIDAR
#include <Wire.h>
#include <VL53L0X.h>
//VL53L0X sensor; ?????????????????????????????????????????????????????????????????


Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

//int pos = 0;    // variable to store the servo position

int distance = 0;
//8191 max distance -> NO OBJECT detected

const int MAX_DISTANCE = 100;

int angle = 0;

bool modeLook = true; //Booleans for mode checking
bool modeTrack = false;
bool objectDetected = false;

int check = 0; //variable used to count the times a failureoccurs when trying to find an object in track mode
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//================================
void setup() {
  //=======================From single.ino
  Serial.begin(115200);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif
  //=======================================

  //=========================From sweep.ino

  myservo.attach(3);  // attaches the servo on pin 9 to the servo object
  //======================================
}

void loop() {
  // put your main code here, to run repeatedly:

  //=======================From single.ino
  // Serial.print(sensor.readRangeSingleMillimeters());



  /* if (sensor.timeoutOccurred()) {
     Serial.print(" TIMEOUT");
    }

    Serial.println(); */
  //=========================


  //=========================From sweep.ino


  //==============LOOK MODE BEGIN======================================================================================//
  for (int pos = 0 + angle; pos <= 180 && modeLook; pos += 1) // goes from 0 degrees to 180 degrees in steps of 1 degree
  {
//    motorControl(400,800);
    distance = sensor.readRangeSingleMillimeters();
    myservo.write(pos);              // tell servo to go to position in variable 'pos'

    Serial.print("Current Angle: ");
    Serial.println(pos);

    if (distance < MAX_DISTANCE) //Object detected
    {
      objectDetected = true;
      modeLook = false;
      modeTrack = true;
      angle = pos;
      Serial.print("OBJECT DETECTED AT ANGLE & DISTANCE: ");
      Serial.print(angle);
      Serial.print(", ");
      Serial.print(distance - 30);
      Serial.println("mm");
    }
    else {
      angle = 0;
    }
    // Serial.println(pos);

    delay(10);                       // waits in ms for the servo to reach the position
  }

  for (int pos = 180 - angle ; pos >= 0 && modeLook; pos -= 1) // goes from 180 degrees to 0 degrees
  {
    distance = sensor.readRangeSingleMillimeters();
    myservo.write(pos);              // tell servo to go to position in variable 'pos'

    Serial.print("Current Angle: ");
    Serial.println(pos);

    if (distance < MAX_DISTANCE) //object detected
    {
      objectDetected = true;
      modeLook = false;
      modeTrack = true;
      angle = pos;
      Serial.print("OBJECT DETECTED AT ANGLE & DISTANCE: ");
      Serial.print(angle);
      Serial.print(", ");
      Serial.print(distance - 30);
      Serial.println("mm");
    }

    else {
      angle = 0;
    }
    //  Serial.println(pos);
    delay(10);                       // waits in ms for the servo to reach the position

  }

  //===================LOOK MODE END========================================================================================//

  //===================TRACK MODE BEGIN====================================================================================//
  if (modeTrack && check < 3) //triggers when object was detected and count variable is less than 3
  {
    Serial.println("Tracking Mode Activated");

    if  (objectDetected) //triggers first time, might be modified later
    {
      angle = 0;
      delay(5000);
    }
    objectDetected = false; //set false since object must be detected again in track mode


    //Do boat stuff
    //angle = 60;

    for (int pos = 60 + angle; pos <= 120 && !objectDetected ; pos += 1) { // goes from 60 degrees to 120 degrees in steps of 1 degree

      //    Serial.print("Inside Tracking Loop of 60 Degrees"); //DEBUGGING PURPOSES

      Serial.print("Current Angle: ");
      Serial.println(pos);
      distance = sensor.readRangeSingleMillimeters();
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      if (distance < MAX_DISTANCE) {
        objectDetected = true;
        modeLook = false;
        modeTrack = true;
        angle = pos;
        check = 0;
        Serial.print("OBJECT DETECTED AT ANGLE & DISTANCE: ");
        Serial.print(angle);
        Serial.print(", ");
        Serial.print(distance - 30);
        Serial.println("mm");


        
      }   
      //  else{angle = 0;}
      // Serial.println(pos);

      delay(10);                       // waits in ms for the servo to reach the position
    }
    for (int pos = 120 - angle ; pos >= 60 && !objectDetected; pos -= 1) { // goes from 120 degrees to 60 degrees
      //   Serial.println("Inside Tracking Loop of 60 Degrees");

      Serial.print("Current Angle: ");
      Serial.println(pos);
      distance = sensor.readRangeSingleMillimeters();
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      if (distance < MAX_DISTANCE) {
        objectDetected = true;
        modeLook = false;
        modeTrack = true;
        angle = pos;
        check = 0;
        Serial.print("OBJECT DETECTED AT ANGLE & DISTANCE: ");
        Serial.print(angle);
        Serial.print(", ");
        Serial.print(distance - 30);
        Serial.println("mm");
      }

      else if (pos == 60) {
        check++;
      }
      //  Serial.println(pos);
      delay(10);


    }

    if  (check == 3)
    {
      modeTrack = false;
      modeLook = true;
      angle = 0;
      check = 0;

    }

  }

  //===================TRACK MODE END========================================================================================//
}
/*
  for (int pos = 0; pos <= 90 && distance < MAX_DISTANCE; pos += 1)
  {
    myservo.write(pos);
    delay(10);
  }

  for (int pos = 90 ; pos >= 0 && distance > MAX_DISTANCE; pos -= 1)
  {
    myservo.write(pos);
    delay(10);
  }
*/
//=========================
