//=======================From single.ino
/* This example shows how to get single-shot range
  measurements from the VL53L0X. The sensor can optionally be
  configured with different ranging profiles, as described in
  the VL53L0X API user manual, to get better performance for
  a certain application. This code is based on the four
  "SingleRanging" examples in the VL53L0X API.

  The range readings are in units of mm. */

#include "NewPing.h"
#include "MedianFilter.h"
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

#define TRIGGER_PIN  6  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     5  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE_SONAR 450 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE_SONAR); // NewPing setup of pins and maximum distance.
MedianFilter filter(31, 0);

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

#include <Servo.h>


Servo myservo;  // create servo object to control a servo


//Global Variables
const int MAX_DISTANCE_LOOK = 1000;   //LIDAR max disatnce when in look mode
const int MAX_DISTANCE_PICKUP = 300; //LIDAR max disatnce when in pickup mode
const int NUM_OF_CHECKS = 1;

int distance = 0;                     //LIDAR distance measurement
int angle = 0;                        //Angle variable that is used as an offset for servo arm swing
//int check = 0;                        //variable used to count the times a failure occurs when trying to find an object in track mode
int distCheckPick = 0;
int distCheckLook = 0;
bool modeLook = true;                 //Booleans for mode checking
bool modeTrack = false;
bool objectDetected = false;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//================================
void setup() {

  Serial.begin(115200);
  Wire.begin();
  sensor.init();
  sensor.setTimeout(500);

  //Servo pin declaration
  myservo.attach(4);

  //Setup Channel A
  pinMode(12, OUTPUT); //Initiates Motor Channel A pin
  pinMode(9, OUTPUT); //Initiates Brake Channel A pin

  //Setup Channel B
  pinMode(13, OUTPUT); //Initiates Motor Channel B pin
  pinMode(8, OUTPUT);  //Initiates Brake Channel B pin



  //LIDAR MODES
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


}

void loop() {
  // put your main code here, to run repeatedly:

  forward();

  if (modeLook)
  {
    lookMode();
  }

  /*
    if (modeTrack && check < 3)
    {
      trackMode();
    }
  */

  delay(50);  // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.

}


//Movement Functions
void forward()
{
  //Motor A forward @ full speed RIGHT
  digitalWrite(12, HIGH);  //Establishes forward direction of Channel A
  analogWrite(3, 200);    //Spins the motor on Channel A at half speed

  //Motor B forward @ full speed LEFT
  digitalWrite(13, HIGH); //Establishes forward direction of Channel B
  analogWrite(11, 255);   //Spins the motor on Channel B at full speed
}

void halt()
{
  digitalWrite(3, 0); //Engage break for Motor A
  digitalWrite(11, 0); //Engage break for Motor B
}

void right()
{
  digitalWrite(12, HIGH);  //Establishes forward direction of Channel A
  analogWrite(3, 255);    //Spins the motor on Channel A at half speed
}

void left()
{
  digitalWrite(11, 0);
  digitalWrite(3, 255);
  delay(5000);
}

void rotate(int x)
{
  Serial.println("ROTATING");

  if (x < 90)
  {
    digitalWrite(11, 0);
    digitalWrite(3, 143); //might need to change this values since the speed is different

    delay(25 * (90 - x));
  }

  if (x >= 90)
  {
    digitalWrite(3, 0);
    digitalWrite(11, 137);
    delay(25 * (x - 90));
  }

}

/*
  void checkPing()
  {
   unsigned int o, uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
   filter.in(uS);
    o = filter.out();
    Serial.print("Ping: ");
    Serial.print( o / US_ROUNDTRIP_CM); // Convert ping time to distance in cm and print result (0 = outside set distance range)
    Serial.println("cm");

    if  (o / US_ROUNDTRIP_CM < 25 && o / US_ROUNDTRIP_CM != 0)
    {left();}

    forward();
  }
*/

//Mode functions

int getLidarDistance()
{
  return sensor.readRangeSingleMillimeters();
}

int lidarCheckPick(int dist)
{
  if (dist <  MAX_DISTANCE_PICKUP)
  {
    return 1;
  }
  return 0;
}


int lidarCheckLook(int dist)
{
  if (dist > MAX_DISTANCE_PICKUP && dist <  MAX_DISTANCE_LOOK)
  {
    return 1;
  }

  return 0;

}

void lookMode()
{
  distCheckPick = 0;
  distCheckLook = 0;
  for (int pos = 30; pos <= 150; pos += 2) // goes from 0 degrees to 180 degrees in steps of 1 degree
  {
    myservo.write(pos);              // tell servo to go to position in variable 'pos'

    Serial.print("Current Angle: ");
    Serial.println(pos);

    distCheckPick = distCheckPick + lidarCheckPick(getLidarDistance());
    distCheckLook =  distCheckLook + lidarCheckLook(getLidarDistance());
    Serial.println(getLidarDistance());
    if (distCheckPick > NUM_OF_CHECKS) //object less than a meter
    {
      Serial.println("1 Meter Object");
      rotate(pos);
      myservo.write(180); //swing arm to look at object
      forward();
      delay(7000);
      return;
    }

/*
    if (distCheckLook > NUM_OF_CHECKS) //Object detected
    {
      Serial.println("Object detected");
      rotate(pos);
      //     delay(4000); //FIND FORWARD SPEED OF BOAT
      trackTest();
      return;
    }
*/
    delay(10);                       // waits in ms for the servo to reach the position
  }
  distCheckPick = 0;
  distCheckLook = 0;

  for (int pos = 150 ; pos >= 30; pos -= 2) // goes from 180 degrees to 0 degrees
  {
    myservo.write(pos);              // tell servo to go to position in variable 'pos'

    Serial.print("Current Angle: ");
    Serial.println(pos);

    distCheckPick = distCheckPick + lidarCheckPick(getLidarDistance());
    distCheckLook =  distCheckLook + lidarCheckLook(getLidarDistance());

    if (distCheckPick > NUM_OF_CHECKS)
    {
      myservo.write(180); //swings arm out of the opening's way
      rotate(pos);
      forward();
      delay(7000);
      return;
    }
    /*
    if (distCheckLook > NUM_OF_CHECKS) //Object detected
    {

      rotate(pos);
      halt();
      //     delay(4000); //FIND FORWARD SPEED OF BOAT
      trackTest();
      return;
    } */
    /*
      distCheckLook = distCheckLook + lidarCheckLook(distance);
      if (distCheckLook > 1) //Object detected
      {
      Serial.print("OBJECT DETECTED AT ANGLE & DISTANCE: ");
      Serial.print(angle);
      Serial.print(", ");
      Serial.print(distance);
      Serial.println("mm");
      rotate(pos);
      trackTest();
      angle = pos;
      return;

      }
    */


    //  Serial.println(pos);
    delay(10);                       // waits in ms for the servo to reach the position

  }


}

void trackTest()
{

  halt();
  Serial.println("STOP");

  int check = 0;
  // Serial.println("Track Function");

  while (check < 3)
  {
    distCheckPick = 0;
    distCheckLook = 0;
    for (int pos = 60; pos <= 120; pos += 1)
    {

      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      //      Serial.print("Current Angle: ");
      //      Serial.println(pos);

      distCheckPick = distCheckPick + lidarCheckPick(getLidarDistance());
      distCheckLook =  distCheckLook + lidarCheckLook(getLidarDistance());

      if (distCheckPick > NUM_OF_CHECKS )
      {
        myservo.write(180); //swings arm out of the opening's way
        rotate(pos);
        forward();
        delay(4000);
        return;
      }

      if (distCheckLook > NUM_OF_CHECKS) //Object detected
      {
        myservo.write(180); //swings arm out of the opening's way
        rotate(pos);
        forward();
        delay(5000); //FIND FORWARD SPEED OF BOAT
        return;
      }

    }
    distCheckPick = 0;
    distCheckLook = 0;

    for (int pos = 120; pos >= 60; pos -= 1)
    {
      distance = sensor.readRangeSingleMillimeters();
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      //    Serial.print("Current Angle: ");
      //   Serial.println(pos);
      distCheckPick = distCheckPick + lidarCheckPick(getLidarDistance());
      distCheckLook =  distCheckLook + lidarCheckLook(getLidarDistance());
      if (distCheckPick > NUM_OF_CHECKS)
      {
        myservo.write(180); //swings arm out of the opening's way
        rotate(pos);
        forward();
        delay(4000);
        return;
      }

      if (distCheckLook > NUM_OF_CHECKS) //Object detected
      {
        myservo.write(180); //swings arm out of the opening's way
        rotate(pos);
        forward();
        delay(5000); //FIND FORWARD SPEED OF BOAT
        return;
      }
    }

    check = check + 1;
  }
}

void trackMode()
{

  int check = 0;
  Serial.println("Tracking Mode Activated");

  if  (objectDetected) //triggers first time, might be modified later
  {
    angle = 0;
    delay(5000);
  }
  objectDetected = false; //set false since object must be detected again in track mode


  for (int pos = 60 + angle; pos <= 120 && !objectDetected ; pos += 1) { // goes from 60 degrees to 120 degrees in steps of 1 degree

    //    Serial.print("Inside Tracking Loop of 60 Degrees"); //DEBUGGING PURPOSES
    //////////////////////////////////////////////////////////////////
    Serial.print("Current Angle: ");
    Serial.println(pos);
    distance = sensor.readRangeSingleMillimeters();;

    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    if (distance < MAX_DISTANCE_PICKUP) {
      //      checkPing();
      objectDetected = true;
      modeLook = false;
      modeTrack = true;
      angle = pos;
      check = 0;
      rotate(pos);
      Serial.print("OBJECT DETECTED AT ANGLE & DISTANCE: ");
      Serial.print(angle);
      Serial.print(", ");
      Serial.print(distance);
      Serial.println("mm");
      unsigned int o, uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).

      filter.in(uS);
      o = filter.out();
      Serial.print("Ping: ");
      Serial.print( o / US_ROUNDTRIP_CM); // Convert ping time to distance in cm and print result (0 = outside set distance range)
      Serial.println("cm");



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

    if (distance < MAX_DISTANCE_PICKUP) {
      objectDetected = true;
      modeLook = false;
      modeTrack = true;
      angle = pos;
      check = 0;
      rotate(pos);
      Serial.print("OBJECT DETECTED AT ANGLE & DISTANCE: ");
      Serial.print(angle);
      Serial.print(", ");
      Serial.print(distance);
      Serial.println("mm");
    }

    else if (pos == 60) {
      check++;
    }
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



