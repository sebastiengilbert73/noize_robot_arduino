#include <StandardCplusplus.h>
#include <sstream>
#include "KeyValueMessage.h"
#include <Adafruit_NeoPixel.h>
#include "AFMotor.h"
#include <Servo.h>

const String version = "TrackedRobot3 V2016-04-14";

// Heartbeat
const int heartbeatPin = 22;
const double heartbeatNormalPeriod = 1.0;
const double heartbeatMessagePeriod = 0.5;
double heartbeatPeriod = heartbeatNormalPeriod;
unsigned long lastHeartbeatChange;
bool heartbeatState;

// KeyValueMessage
KeyValueMessage interpret;
const char endOfMessageCharacter = '|';

// DC Motors
AF_DCMotor motorLeft(1, MOTOR12_64KHZ);
AF_DCMotor motorRight(2, MOTOR12_64KHZ);

// Servo motors
Servo neckServo;
int neckServoTheta;
int neckServoPin = 9;

// Contact switches
const int frontLeftContactSwitchPin = 24;
const int frontRightContactSwitchPin = 52;
unsigned long lastFrontContactSwitchEvent;
const double frontContactSwitchNoRepeatDelay = 0.2;

// Headlights
const int headlightsPin = 53;
Adafruit_NeoPixel headlights = Adafruit_NeoPixel(2, headlightsPin);

bool isDebug = true;

using namespace std;

void setup()
{
  Serial.begin(115200);
  Serial.println("Running " + version);
  // Serial communication with Serial2 port
  Serial2.begin(115200);
  
  // Heartbeat
  pinMode(heartbeatPin, OUTPUT);
  lastHeartbeatChange = millis();
  heartbeatState = true;
  digitalWrite(heartbeatPin, heartbeatState);
  
  // DC Motors
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
  
  // Servos
  neckServo.attach(neckServoPin);
  neckServoTheta = 90;
  neckServo.write(neckServoTheta);
  
  // Contact switches
  pinMode(frontLeftContactSwitchPin, INPUT);
  pinMode(frontRightContactSwitchPin, INPUT);
  lastFrontContactSwitchEvent = millis();
  
  // Headlights
  headlights.begin();
  headlights.show();
}
  
void loop()
{
  unsigned long currentMillis = millis();
  // Heartbeat
  if (currentMillis - lastHeartbeatChange > 500 * heartbeatPeriod)
  {
    heartbeatState = !heartbeatState;
    digitalWrite(heartbeatPin, heartbeatState);
    lastHeartbeatChange = currentMillis;
  }

  if (Serial2.available() > 0)
  {
    ReceiveMessage();
  }
  
  
  // Emit sensor signals
  bool frontLeftContactSwitchEvent = digitalRead(frontLeftContactSwitchPin) == HIGH;
  bool frontRightContactSwitchEvent = digitalRead(frontRightContactSwitchPin) == HIGH;

  if ( (frontLeftContactSwitchEvent || frontRightContactSwitchEvent) &&
      currentMillis - lastFrontContactSwitchEvent > 1000 * frontContactSwitchNoRepeatDelay)
  {
    String msgStr = "";
    if (frontLeftContactSwitchEvent && frontRightContactSwitchEvent)
    {
      msgStr = interpret.KeyValueString("frontLeftContactSwitch", "high") + " " + interpret.KeyValueString("frontRightContactSwitch", "high") + " " + endOfMessageCharacter;
    }
    else if (frontLeftContactSwitchEvent)
    {
      msgStr = interpret.KeyValueString("frontLeftContactSwitch", "high") + " " + endOfMessageCharacter;
    }
    else
    {
      msgStr = interpret.KeyValueString("frontRightContactSwitch", "high") + " " + endOfMessageCharacter;
    }
    lastFrontContactSwitchEvent = currentMillis;

    Serial2.println(msgStr);
    if (isDebug)
    {
      Serial.println(msgStr);
    }    
  }
 
}

void ReceiveMessage()
{
  String msgStr = Serial2.readStringUntil(endOfMessageCharacter);
  
  if (isDebug)
  {
    Serial.print("ReceiveMessage(): ");
    Serial.println(msgStr);
    
  }

  int valueInt = 0;
  if (interpret.ValueAsInt("moveForward", msgStr, &valueInt))
  {
    RunMotors(valueInt, valueInt);
  }
  if (interpret.ValueAsInt("stop", msgStr, &valueInt))
  {
    RunMotors(0, 0);
  }
  if (interpret.ValueAsInt("turnLeft", msgStr, &valueInt))
  {
    RunMotors(-valueInt, valueInt);
  }
  if (interpret.ValueAsInt("turnRight", msgStr, &valueInt))
  {
    RunMotors(valueInt, -valueInt);
  }
  int leftMotorValue = -9999, rightMotorValue = -9999;
  if (interpret.ValueAsInt("leftMotor", msgStr, &leftMotorValue) | 
   interpret.ValueAsInt("rightMotor", msgStr, &rightMotorValue) )
  {
    if (leftMotorValue == -9999)
       leftMotorValue = 0;
    if (rightMotorValue == -9999)
       rightMotorValue = 0;
    RunMotors(leftMotorValue, rightMotorValue);
  }
  
  
  if (interpret.KeyIsPresent("headlights", msgStr))
  {
    if (interpret.ValueOf("headlights", msgStr) == "off")
    {
      headlights.setPixelColor(0, 0, 0, 0); // ledNdx, R, G, B
      headlights.setPixelColor(1, 0, 0, 0); // ledNdx, R, G, B
      headlights.show();
    }
    else // headlights=on
    {
      int leftRed = 0, leftGreen = 0, leftBlue = 0, rightRed = 0, rightGreen = 0, rightBlue = 0;
      interpret.ValueAsInt("leftRed", msgStr, &leftRed);
      interpret.ValueAsInt("leftGreen", msgStr, &leftGreen);
      interpret.ValueAsInt("leftBlue", msgStr, &leftBlue);
      interpret.ValueAsInt("rightRed", msgStr, &rightRed);
      interpret.ValueAsInt("rightGreen", msgStr, &rightGreen);
      interpret.ValueAsInt("rightBlue", msgStr, &rightBlue);
      headlights.setPixelColor(0, leftRed, leftGreen, leftBlue);
      headlights.setPixelColor(1, rightRed, rightGreen, rightBlue);
      headlights.show();
    }
  }
  
  if (interpret.KeyIsPresent("lookCenter", msgStr) )
  {
    neckServoTheta = 90;
    neckServo.write(neckServoTheta);
  }
  int deltaTheta = 0;
  if (interpret.ValueAsInt("lookLeft", msgStr, &deltaTheta) )
  {
    neckServoTheta = neckServoTheta - deltaTheta;
    neckServo.write(neckServoTheta);
  }
  if (interpret.ValueAsInt("lookRight", msgStr, &deltaTheta) )
  {
    neckServoTheta = neckServoTheta + deltaTheta;
    neckServo.write(neckServoTheta);
  }
}

void RunMotors(int leftMotorSpeed, int rightMotorSpeed)
{
  if (leftMotorSpeed >= 0)
  {
    motorLeft.setSpeed(leftMotorSpeed);
    motorLeft.run(FORWARD);
  }
  else
  {
    motorLeft.setSpeed(-leftMotorSpeed);
    motorLeft.run(BACKWARD);
  }
  if (rightMotorSpeed >= 0)
  {
    motorRight.setSpeed(rightMotorSpeed);
    motorRight.run(FORWARD);
  }
  else
  {
    motorRight.setSpeed(-rightMotorSpeed);
    motorRight.run(BACKWARD);
  }
}
  
