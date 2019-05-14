//=============SONAR START===================//
#include "NewPing.h"
#include "MedianFilter.h"
#include <Wire.h>

#define TRIGGER_PIN  14  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     12  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE_SONAR 450 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE_SONAR); // NewPing setup of pins and maximum distance.
MedianFilter filter(31, 0);
//=============SONAR END===================//
#include <ESP8266WiFi.h>
//#include <BlynkSimpleEsp8266.h>
//=============LIDAR START===================//
#include <VL53L0X.h>
#include <Servo.h>

VL53L0X sensor;
Servo myservo;  // create servo object to control a servo
//int pos = 0;    // variable to store the servo position

int distance = 0; 
const int MAX_DISTANCE_LIDAR = 100; //8191 max distance -> NO OBJECT detected
int angle = 0;

bool modeLook = true; //Booleans for mode checking
bool modeTrack = false;
bool objectDetected = false;
int check = 0; //variable used to count the times a failure occurs when trying to find an object in track mode
//=============LIDAR END===================//

//=============MOTOR START===================//
#define RightMotorSpeed 5
#define RightMotorDir   0
#define LeftMotorSpeed  4
#define LeftMotorDir    2

//char auth[] = "41db1537ca7b4e3d8bb064fe62c6373d";
//char ssid[] = "OnePlus 6";
//char pass[] = "ForzaInter";

int minRange = 312;
int maxRange = 712;

int minSpeed = 450;
int maxSpeed = 1020;
int noSpeed = 0;
//=============MOTOR END===================//

//==============SUPLEMENTAL CODE START=====================//
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
//==============SUPLEMENTAL CODE END=====================//




void setup()
{

  //====================LIDAR START==================
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
  //====================LIDAR END==================


  //====================MOTOR START==================

  //  Blynk.begin(auth, ssid, pass);
  // initial settings for motors off and direction forward
  pinMode(RightMotorSpeed, OUTPUT);
  pinMode(LeftMotorSpeed, OUTPUT);
  pinMode(RightMotorDir, OUTPUT);
  pinMode(LeftMotorDir, OUTPUT);

  digitalWrite(RightMotorSpeed, LOW);
  digitalWrite(LeftMotorSpeed, LOW);
  digitalWrite(RightMotorDir, HIGH);
  digitalWrite(LeftMotorDir, HIGH);
    //====================MOTOR END==================


}


void loop()
{
  //Code Starts here:
  

  

  



  //=============SONAR START===================//

  //=============SONAR END===================//

  //=============LIDAR START===================//

  //==============LOOK MODE BEGIN======================================================================================//

  //===================LOOK MODE END========================================================================================//

  //===================TRACK MODE BEGIN====================================================================================//

  //===================TRACK MODE END========================================================================================//

  //=============LIDAR END===================================================================================================//
}


//=================LIDAR FUNCTION START===============================================
void lidarControl(bool state) //state variable will start or end lidar arm movement
{
//==============LOOK MODE BEGIN======================================================================================//
  for (int pos = 0 + angle; pos <= 180 && modeLook; pos += 1) // goes from 0 degrees to 180 degrees in steps of 1 degree
  {
    distance = sensor.readRangeSingleMillimeters();
    myservo.write(pos);              // tell servo to go to position in variable 'pos'

    Serial.print("Current Angle: ");
    Serial.println(pos);

    if (distance < MAX_DISTANCE_LIDAR) //Object detected
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

    if (distance < MAX_DISTANCE_LIDAR) //object detected
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
      if (distance < MAX_DISTANCE_LIDAR) {
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
      if (distance < MAX_DISTANCE_LIDAR) {
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
//=================LIDAR FUNCTION END=================================================

//=================SONAR FUNCTION START===============================================
void sonarControl(bool state) //state variable will start or end sensor readings
{
    delay(50);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned int a, uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).

  filter.in(uS); //filtering
  a = filter.out();

  if ((a / US_ROUNDTRIP_CM) > 22 || ( a / US_ROUNDTRIP_CM) == 0) //22 is chosen as min value
  {
    moveControl(400, 800); // Calls function for motor moevement
    Serial.println("Moving Forward");
    Serial.print("Ping: ");
    Serial.print( a / US_ROUNDTRIP_CM); // Convert ping time to distance in cm and print result (0 = outside set distance range)
    Serial.println("cm");

  }

  else if ( (a / US_ROUNDTRIP_CM ) != 0) {

    Serial.println("Obstacle Detected: Turn Boat");
    moveControl(800, 800); // Calls function for motor moevement
    delay(25);
  }
}
//=================SONAR FUNCTION END===============================================

//===================MOTOR FUNCTION START==================//
void moveControl(int x, int y)
{

  if (y >= maxRange && x >= minRange && x <= maxRange) //zataci R
  {
    digitalWrite(RightMotorDir, HIGH);
    digitalWrite(LeftMotorDir, HIGH);
    analogWrite(RightMotorSpeed, maxSpeed);
    analogWrite(LeftMotorSpeed, maxSpeed);
  }
  /*
    // move forward right
    else if(x >= maxRange && y >= maxRange)   //zataci R
    {
      digitalWrite(RightMotorDir,HIGH);
      digitalWrite(LeftMotorDir,HIGH);
     analogWrite(RightMotorSpeed,minSpeed);
      analogWrite(LeftMotorSpeed,maxSpeed);
    }
  */
  // move forward left
  else if (x <= minRange && y >= maxRange)
  {
    digitalWrite(RightMotorDir, HIGH);
    digitalWrite(LeftMotorDir, HIGH);
    analogWrite(RightMotorSpeed, maxSpeed);
    analogWrite(LeftMotorSpeed, minSpeed);
  }
  /*
    //   neutral zone
    else if(y < maxRange && y > minRange && x < maxRange && x > minRange)
    {
      analogWrite(RightMotorSpeed,noSpeed);
      analogWrite(LeftMotorSpeed,noSpeed);
    }

    // move back
    else if(y <= minRange && x >= minRange && x <= maxRange)
    {
      digitalWrite(RightMotorDir,LOW);
      digitalWrite(LeftMotorDir,LOW);
     analogWrite(RightMotorSpeed,maxSpeed);
      analogWrite(LeftMotorSpeed,maxSpeed);
    }

    // move back and right
    else if(y <= minRange && x <= minRange)
    {
     digitalWrite(RightMotorDir,LOW);
      digitalWrite(LeftMotorDir,LOW);
      analogWrite(RightMotorSpeed,minSpeed);
      analogWrite(LeftMotorSpeed,maxSpeed);
    }

    // move back and left
    else if(y <= minRange && x >= maxRange)
    {
      digitalWrite(RightMotorDir,LOW);
      digitalWrite(LeftMotorDir,LOW);
      analogWrite(RightMotorSpeed,maxSpeed);
      analogWrite(LeftMotorSpeed,minSpeed);
    }

  */

}
//===================MOTOR FUNCTION END==================//
