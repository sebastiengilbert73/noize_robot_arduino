#include <StandardCplusplus.h>
#include <sstream>
#include "KeyValueMessage.h"
#include <Adafruit_NeoPixel.h>


const String version = "TrackedRobot3 V2016-04-06";

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
}
  
