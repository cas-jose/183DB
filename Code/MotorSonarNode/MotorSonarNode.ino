// ---------------------------------------------------------------------------
// Example NewPing library sketch that does a ping about 20 times per second.
// ---------------------------------------------------------------------------


#include "NewPing.h"
#include "MedianFilter.h"
#include <Wire.h>


#define TRIGGER_PIN  D4  // Arduino pin tied to trigger pin on the ultrasonic sensor. //PINS WORK, DO NOT CHANGE
#define ECHO_PIN     D5  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 450 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

MedianFilter filter(31,0);

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


void setup() {
  
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.


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

void loop() {
  
  delay(50);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned int a,uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).

  filter.in(uS);
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
    moveControl(200, 800); // Calls function for motor moevement
    delay(50);
  }

  
}

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
  
    // move forward right
    else if(x >= maxRange && y >= maxRange)   //zataci R
    {
      digitalWrite(RightMotorDir,HIGH);
      digitalWrite(LeftMotorDir,HIGH);
     analogWrite(RightMotorSpeed,minSpeed);
      analogWrite(LeftMotorSpeed,maxSpeed);
    }
  
  // move forward left
  else if (x <= minRange && y >= maxRange)
  {
    digitalWrite(RightMotorDir, HIGH);
    digitalWrite(LeftMotorDir, HIGH);
    analogWrite(RightMotorSpeed, maxSpeed);
    analogWrite(LeftMotorSpeed, minSpeed);
  }
  
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

  

}
//===================MOTOR FUNCTION END==================//
